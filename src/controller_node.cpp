#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "control.h"
#include "kinematic_model.h"
#include "trajectory_generation.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>

using namespace std::chrono_literals;
using std::cout;
using std::cin;
using std::endl;
using std::placeholders::_1;

/* FRANKA CONTROLLER NDOE
 * Subscribe to desired cartesian pose
 * Publish desired motor position
 */
class ControllerNode : public rclcpp::Node {
  public:
    ControllerNode() : Node("controller_node") {
      this->subPose = this->create_subscription<std_msgs::msg::Float64MultiArray>("/desired_pos", 10, std::bind(&ControllerNode::topic_callback, this, _1));
      this->joint1Pub = this->create_publisher<std_msgs::msg::Float64>("/joint1Pos", 10);
      this->joint2Pub = this->create_publisher<std_msgs::msg::Float64>("/joint2Pos", 10);
      this->joint3Pub = this->create_publisher<std_msgs::msg::Float64>("/joint3Pos", 10);
      this->joint4Pub = this->create_publisher<std_msgs::msg::Float64>("/joint4Pos", 10);
      this->joint5Pub = this->create_publisher<std_msgs::msg::Float64>("/joint5Pos", 10);
      this->joint6Pub = this->create_publisher<std_msgs::msg::Float64>("/joint6Pos", 10);
      this->joint7Pub = this->create_publisher<std_msgs::msg::Float64>("/joint7Pos", 10);
      this->timer = this->create_wall_timer(10ms, std::bind(&ControllerNode::timer_callback, this));

      this->moving = false;
      this->t = 0;
      this->dt = 1e-3;

      /* Setting up Robot Model and Controller */
      this->a = std::vector<double>({0.0, 0.0, 0.0, 0.0825, -0.0825, 0, 0.088, 0});
      this->d = std::vector<double>({0.333, 0.0, 0.316, 0, 0.384, 0, 0, 0.107});
      this->alpha = std::vector<double>({0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0});
      
      this->rm = RobotModel(this->a, this->d, this->alpha);
      this->controller = Controller(rm);
    }
  
  private:
    void timer_callback() {
      if(this->moving) {
        Eigen::Vector3d xOut;
        Eigen::Matrix3d ROut;
        Eigen::Matrix<double, 6, 7> JOut;

        /* Get next position */
        Eigen::Vector3d Xp2p = this->p2p.X(t);
        rm.FwdKin(xOut, ROut, JOut, this->q);
        Eigen::Vector3d Xrm = xOut;

        /* Velocity vector (x, y, z) in time t */
        Eigen::Vector3d DXd_ff = p2p.dX(t);

        /* Position vector (x, y, z) in time t + dt */
        Eigen::Vector3d Xd = p2p.X(t+dt);

        /* Orientation vector (ox, oy, oz) in robot model */
        Eigen::Vector3d yaw_pitch_roll = ROut.eulerAngles(0, 1, 2);
        double theta_x = yaw_pitch_roll(0);
        double theta_y = yaw_pitch_roll(1);
        double theta_z = yaw_pitch_roll(2);
        Eigen::Vector3d Orm = {theta_x, theta_y, theta_z};

        /* Orientation vector (ox, oy, oz) in time t */
        Eigen::Matrix3d R = p2p.Q(t).toRotationMatrix();
        yaw_pitch_roll = R.eulerAngles(0, 1, 2);
        theta_x = yaw_pitch_roll(0);
        theta_y = yaw_pitch_roll(1);
        theta_z = yaw_pitch_roll(2);
        Eigen::Vector3d O = {theta_x, theta_y, theta_z};

        /* Orientation vector (ox, oy, oz) in time t+dt */
        Eigen::Matrix3d Rd = p2p.Q(t+dt).toRotationMatrix();
        yaw_pitch_roll = Rd.eulerAngles(0, 1, 2);
        theta_x = yaw_pitch_roll(0);
        theta_y = yaw_pitch_roll(1);
        theta_z = yaw_pitch_roll(2);
        Eigen::Vector3d Od = {theta_x, theta_y, theta_z};

        /* R'(t) */
        Eigen::Matrix3d dR = Rd - R;

        /* S(w) */ 
        Eigen::Matrix3d Sw = dR*(R.inverse()); 

        /* Feed forward angular velocity */
        Eigen::Vector3d W_ff = {Sw(2, 1), Sw(0, 2), Sw(1, 0)};

        /* Joint variable velocity for trajectory constraints */
        Eigen::Vector<double, 7> dq = controller.Dqd(q, Xd, DXd_ff, Rd, W_ff);

        this->q = this->q + dq*this->dt;

        cout << "================" << endl;
        cout << "t = " << t <<endl;
        cout << "Xp2p = " << Xp2p.transpose() << endl;
        cout << "Xrm = " << Xrm.transpose() << endl;
        cout << "Xd = " << Xd.transpose() << endl;
        cout << "DXd_ff = " << DXd_ff.transpose() << endl;
        cout << "DXd_ff_A = " << ((Xd - Xp2p)/dt).transpose() << endl;
        cout << "O = " << O.transpose() << endl;
        cout << "Orm = " << Orm.transpose() << endl;
        cout << "Od = " << Od.transpose() << endl;
        cout << "W_ff = " << W_ff.transpose() << endl;
        cout << "q = " << this->q.transpose() << endl;
        cout << "dq = " << dq.transpose()*dt << endl;
        cout << endl;

        q_log += q;

        /* Update time */
        this->t += this->dt;
        if(this->t >= this->Dt) {
          this->moving = false;
          this->t = 0;

          // cout << "[LOG] Q: " << q_log.transpose()/1000 << endl;
        }
      }
      
      auto msg = std_msgs::msg::Float64();

      /* Joint 1 */
      msg.data = this->q[0];
      joint1Pub->publish(msg);
      /* Joint 2 */
      msg.data = this->q[1];
      joint2Pub->publish(msg);
      /* Joint 3 */
      msg.data = this->q[2];
      joint3Pub->publish(msg);
      /* Joint 2 */
      msg.data = this->q[3];
      joint4Pub->publish(msg);
      /* Joint 2 */
      msg.data = this->q[4];
      joint5Pub->publish(msg);
      /* Joint 2 */
      msg.data = this->q[5];
      joint6Pub->publish(msg);
      /* Joint 7 */
      msg.data = this->q[6];
      joint7Pub->publish(msg);
    }

    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if(msg->data.size() != 6) {
        cout << "[ERROR] Wrong size for Robot pose. Expected array of size 6." << endl;
      } else { 
        this->X[0] = msg->data[0];
        this->X[1] = msg->data[1];
        this->X[2] = msg->data[2];
        this->O[0] = M_PI*msg->data[3]/180;
        this->O[1] = M_PI*msg->data[4]/180;
        this->O[2] = M_PI*msg->data[5]/180;

        q_log = {0, 0, 0, 0, 0, 0, 0};

        /* Trajectory Generation */
        this->generateTrajectory();
      }
    }

    void generateTrajectory() {
      /* Get current pose (xOut, ROut)*/
      Eigen::Vector3d xOut;
      Eigen::Matrix3d ROut;
      Eigen::Matrix<double, 6, 7> JOut;
      Eigen::Vector<double, 7> q;
      rm.FwdKin(xOut, ROut, JOut, this->q);

      /* Quaternion for current orientation */
      Eigen::Quaterniond qi(ROut);

      /* Motion time lapse */
      // this->Dt = 10*(pfs - pis).norm();
      this->Dt = 1;

      /* Final orientation quaternion */
      Eigen::Quaterniond qf;
      qf = Eigen::AngleAxisd(this->O[0], Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(this->O[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(this->O[2], Eigen::Vector3d::UnitZ());

      // cout << "xf: " << this->X.transpose() << endl;
      // cout << "qf: " << qf.coeffs() << endl;

      /* Point to point generation */
      this->p2p = Point2Point(xOut, qi, this->X, qf, this->Dt, 0);

      /* Start movement */
      this->moving = true;

      this->t = 0;
    }

    /* Pose subscriber */
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subPose;
    /* Joint position publishers */
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint1Pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint2Pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint3Pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint4Pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint5Pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint6Pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint7Pub;
    /* Timer */
    rclcpp::TimerBase::SharedPtr timer;
    /* Robot Pose: (x, y, z, ox, oy, oz) */
    Eigen::Vector<double, 3> X = {0, 0, 0};
    Eigen::Vector<double, 3> O = {0, 0, 0};
    /* Joint configuration */
    Eigen::Vector<double, 7> q = {0, 0, 0, -1.0, 0, 2, 0};
    /* Log of q values */
    Eigen::Vector<double, 7> q_log = {0, 0, 0, 0, 0, 0, 0};

    /* Time variables */
    double t;
    double Dt;
    double dt;
    /* */
    bool moving;
    /* Trajectory Generator -> Point to Point (x, y, z, ox, oy, oz) */
    Point2Point p2p;
    /* Robot Model */
    RobotModel rm;
    /* Controller */
    Controller controller;
    /* Robot features */
    std::vector<double> a;
    std::vector<double> d;
    std::vector<double> alpha;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();

  return 0;
}

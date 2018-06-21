// Author: Akash Patel (apatel435@gatech.edu)
// Purpose: To simulate Krang moving from one pose to another pose
//          i.e. modelling how exactly Krang does this (qdot and
//          qdoubledout[qddot])
//          This code can be used as a basis to ensure the final pose and
//          trajectory is feasible (it doesnt collide with itself) (code does not
//          consider balancing)
//
//          Question: given a final pose, the trajectory taken depends on the
//          intial pose right? so there might be cases where velocity commands
//          need to be split up into multiple commands (by vel com I mean the
//          commands of angular velocities sent for the actuators moving all
//          each joint)
//          there might also be cases where the final pose can never be reached
//          no matter the trajectory based on the intial pose
//
//          ALSO: (this is prob a whole another project and ties into the
//          dynamics of Krang) but,
//          What is the optimal trajectory from one pose to another pose
//
// Input: Krang urdf model, data points (q/poses and qdots qddots) as a file
// Output: A visualization of Krang moving from one pose to another pose as well
//          as a truth value if (initial pose -> velocity commands -> final pose) is
//          feasible
//
// STOP!!!!!
// ONE STEP AT A TIME
// LET'S FOCUS ON DETERMING IF A SINGLE POSE IS FEASIBLE
// BEFORE THAT LET'S SIMULATE A POSE IN DART
// algo lets try to read a pose file and then step through all poses in file in
// DART sim (use a keypress to advance to next pose)
// Could also use this to check collisions systemically (brute force in DART)
// bodyNodeCollisionFilter::areAdjacentBodies

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <nlopt.hpp>

using namespace std;
using namespace dart::collision;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

#define MAXBUFSIZE ((int) 1e6)

struct comOptParams {
  SkeletonPtr robot;
  Eigen::Matrix<double, 25, 1> qInit;
};

double comOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
  Eigen::Matrix<double, 25, 1> q(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 25, 1> mGrad = q-optParams->qInit;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5*pow((q-optParams->qInit).norm(), 2));
}

double comConstraint(const std::vector<double> &x, std::vector<double> &grad, void *com_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(com_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  return (pow(optParams->robot->getCOM()(0)-optParams->robot->getPosition(3), 2) \
    + pow(optParams->robot->getCOM()(1)-optParams->robot->getPosition(4), 2));
}

double wheelAxisConstraint(const std::vector<double> &x, std::vector<double> &grad, void *wheelAxis_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(wheelAxis_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  return optParams->robot->getBodyNode(0)->getTransform().matrix()(2,0);
}

double headingConstraint(const std::vector<double> &x, std::vector<double> &grad, void *heading_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(heading_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  Eigen::Matrix<double, 4, 4> Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
  double heading = atan2(Tf(0,0), -Tf(1,0));
  optParams->robot->setPositions(optParams->qInit);
  Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
  double headingInit = atan2(Tf(0,0), -Tf(1,0));
  return heading-headingInit;
}

Eigen::MatrixXd genDartPoseParams() {

    int controlInputPoses = 10;
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers (the pose params)
    ifstream infile;
    //infile.open("../randomPoses500.txt");
    infile.open("../2_comPoses10.txt");
    while(! infile.eof() && rows <= controlInputPoses) {
        string line;
        getline(infile, line);
        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = rows;
    int numParams = cols;

    Eigen::MatrixXd allInitPoseParamsFromFile(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            allInitPoseParamsFromFile(i,j) = buff[cols*i+j];

    Eigen::MatrixXd allInitPoseParams(cols, rows);
    allInitPoseParams = allInitPoseParamsFromFile.transpose();

    //ofstream dartPoseParamsFile;
    //dartPoseParamsFile.open("dartPoseParams.txt");

    // Instantiate krang
    // There has to be a way to do relative filepaths in DART
    // but until I figure that out I guess this works
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr krang = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    const int totalParams = (const int) krang->getNumDofs();
    // dof should be 25
    // What's the format
    // first three axis-angle (aa)
    // aa1, aa2, aa3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    Eigen::MatrixXd allDartPoseParams(totalParams, numInputPoses);

    for (int pose = 0; pose < numInputPoses; pose++) {
        Eigen::Matrix<double, 24, 1> initPoseParams;
        for (int j = 0; j < cols; j++) {
            initPoseParams(j) = allInitPoseParams(j, pose);
        }

        double headingInit = initPoseParams(0);
        double qBaseInit = initPoseParams(1);
        Eigen::Matrix<double, 22, 1> unchangedValues; unchangedValues << initPoseParams.segment(2,22);

        // Calculating the axis angle representation of orientation from headingInit and qBaseInit:
        // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
        Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

        // Now compile this data into dartPoseParams
        Eigen::Matrix<double, 25, 1> dartPoseParams;
        dartPoseParams << aa.angle()*aa.axis(), unchangedValues;

        // Ensure CoM is right on top of wheel axis
        //const int dof = (const int)krang->getNumDofs();
        //comOptParams optParams;
        //optParams.robot = krang;
        //optParams.qInit << aa.angle()*aa.axis(), unchangedValues;
        //nlopt::opt opt(nlopt::LN_COBYLA, dof);
        //std::vector<double> q_vec(dof);
        //double minf;
        //opt.set_min_objective(comOptFunc, &optParams);
        //opt.add_equality_constraint(comConstraint, &optParams, 1e-8);
        //opt.add_equality_constraint(wheelAxisConstraint, &optParams, 1e-8);
        //opt.add_equality_constraint(headingConstraint, &optParams, 1e-8);
        //opt.set_xtol_rel(1e-4);
        //opt.set_maxtime(10);
        //opt.optimize(q_vec, minf);
        //Eigen::Matrix<double, 25, 1> dartPoseParams(q_vec.data());

        //dartPoseParamsFile << dartPoseParams.transpose() << "\n";

        // Really Dirty i feel bad about writing this
        // TODO
        for (int param = 0; param < totalParams; param++) {
            allDartPoseParams(param, pose) = dartPoseParams(param);
        }

    }

    return allDartPoseParams;

}

class Controller {
    public:
        /// Constructor
        Controller(const SkeletonPtr& krang) : mKrang(krang){
            allPoseParams = genDartPoseParams();
            currPoseParams = Eigen::MatrixXd::Zero(allPoseParams.rows(), 1);
        }

        // TODO: Maybe i can read more poses in the controller update method
        // Controller Update Method
        // Start with initial pose
        // Move a joint every 100 time
        // --- while this is happening check for collisions
        // After moving to final pose go back to initial pose
        // Read next line(pose) and repeat from process 3k
        // Output can print out feasible poses with simple joint order movement
        //
        // Add keyboard input: i.e. press <key> to go back to its initial state
        // and/or press <key> to move the joint
        // Can/should be translated to hardware

        void setNewPose(int time, int scale) {

            int index = time / (1000 * scale);
            int timeStep = time % (1000 * scale);

            // Need to solve issue of having a step that reverts back to zero
            // pose and starts stepping through
            if (timeStep == 0) {
                currPoseParams = Eigen::MatrixXd::Zero(allPoseParams.rows(), 1);
                mKrang->setPositions(currPoseParams);
            }

            if (index < allPoseParams.cols()) {
                Eigen::MatrixXd finalPoseParams = allPoseParams.col(index);

                //40 because we are stepping at 1000 intervals with 25 different
                //pose params (1000/25 = 40)
                int param = timeStep / (40 * scale);
                int step = timeStep % (40 * scale);

                currPoseParams.row(param) = finalPoseParams.row(param) * (step + 1) / (40 * scale);
                mKrang->setPositions(currPoseParams);
            }
        }

        SkeletonPtr getKrang() {
            return mKrang;
        }

    protected:
        //Krang
        SkeletonPtr mKrang;
        //Matrix of the poses
        Eigen::MatrixXd allPoseParams;
        Eigen::MatrixXd currPoseParams;
};

class MyWindow : public SimWindow {

    public:

        // Constructor
        MyWindow(const WorldPtr& world) {
            setWorld(world);
            mController = dart::common::make_unique<Controller>(
                mWorld->getSkeleton("krang"));

        }

        void timeStepping() override {
            //Time in milliseconds
            int worldTime = (int) (mWorld->getTime()*1000);

            // Use controller to move to the next pose based on the time of
            // simulation
            // Time scale to make simulation seem slower/faster
            // Higher value means slower simulation
            int scale = 5;
            mController->setNewPose(worldTime, scale);

            SimWindow::timeStepping();
        }

        // Keyboard input during simulation
        void keyboard(unsigned char _key, int _x, int _y) {

            switch (_key) {
                case 'p': // Print current pose information
                    cout << mController->getKrang()->getPositions().transpose() << "\n";
                    break;
                default:
                    // Default keyboard control
                    SimWindow::keyboard(_key, _x, _y);
                    break;
            }

            //Keyboard control for Controller
            //mController->keyboard(_key, _x, _y);

            glutPostRedisplay();

        }
    protected:
        std::unique_ptr<Controller> mController;

};

dart::dynamics::SkeletonPtr createFloor() {

    dart::dynamics::SkeletonPtr floor = Skeleton::create("floor");

    return floor;

}

dart::dynamics::SkeletonPtr createKrang() {

    // Instantiate krang
    // There has to be a way to do relative filepaths in DART
    // but until I figure that out I guess this works
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr krang = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");
    krang->setName("krang");

    // first three axis-angle (aa)
    // aa1, aa2, aa3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    // Set position of krang
    //krang->setPositions(dartPoseParams);

    return krang;
}

int main(int argc, char* argv[]) {

    // create and initialize the world
    WorldPtr world = World::create();

    // load skeletons
    dart::dynamics::SkeletonPtr floor = createFloor();

    Eigen::MatrixXd allDartPoseParams;
    allDartPoseParams = genDartPoseParams();

    dart::dynamics::SkeletonPtr mKrang = createKrang();

    mKrang->setPositions(allDartPoseParams.col(0));

    int numBodies = mKrang->getNumBodyNodes();
    dart::dynamics::BodyNodePtr bodyi;
    dart::dynamics::BodyNode* acBodyi = bodyi;
    double mi;
    double xmi;
    double ymi;
    double zmi;

    bool isColliding = false;
    CollisionResult colRes;

    for (int i = 0; i < numBodies; i++) {
        bodyi = mKrang->getBodyNode(i);
        dart::dynamics::BodyNode* acBodyi = bodyi;
        //cout << bodyi->getName() << ": ";
        //cout << colRes::inCollision(acBodyi) << "\n";
        //if(acBodyi->isColliding()) {
        //for (int j = 0; j < numBodies; j++) {
            // is a body always colliding the ones attached to it?
            // if so does the pose creation joint limits account for this so
            // that we can assume
            //if (i != j && bodyi.isColliding()) {

                //isColliding = 1;

            //}
        //}
    }

    // add ground and robot to the world pointer
    world->addSkeleton(floor);
    world->addSkeleton(mKrang);

    // no gravity
    Eigen::Vector3d gravity(0.0, 0.0, 0.0);
    world->setGravity(gravity);

    // create a window and link it to the world
    MyWindow window(world);

    glutInit(&argc, argv);
    window.initWindow(960, 720, "Simulate Poses");
    glutMainLoop();

}

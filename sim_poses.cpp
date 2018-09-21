// Author: Akash Patel (apatel435@gatech.edu)
// Purpose: To simulate Krang moving from one pose to another pose
//          i.e. modelling how exactly Krang does this (qdot and
//          qdoubledout[qddot])
//          This code can be used as a basis to ensure the final pose and
//          trajectory is feasible (it doesnt collide with itself)
//
//          Question: given a final pose, the trajectory taken depends on the
//          intial pose right? so there might be cases where velocity commands
//          need to be split up into multiple commands (by vel com I mean the
//          commands of angular velocities sent for the actuators moving all
//          each joint)
//          there might also be cases where the final pose can never be reached
//          no matter the trajectory based on the intial pose
//
// Input: Krang urdf model, data points (q/poses and qdots qddots) as a file
// Output: A visualization of Krang moving from one pose to another pose as well
//          as a truth value if (initial pose -> velocity commands -> final pose) is
//          feasible
//
// STOP!!!!!
// ONE STEP AT A TIME
// LET'S FOCUS ON DETERMING IF A SINGLE POSE IS FEASIBLE
// DART sim (use keypresses to move in the simulation i.e. back a pose forward a
// pose)
// Could also use this to check collisions systemically (brute force in DART)

// Includes
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "../18h-Util/convert_pose_formats.hpp"
#include "../18h-Util/file_ops.hpp"

// Namespaces
using namespace std;
using namespace dart::collision;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::gui;
using namespace dart::math;
using namespace dart::simulation;
using namespace dart::utils;

// Classes
// // Controller
class Controller {
    public:
        /// Constructor
        Controller(const SkeletonPtr& krang, Eigen::MatrixXd poses) : mKrang(krang), inputPoses(poses){
            currPoseParams = Eigen::MatrixXd::Zero(1, inputPoses.cols());
        }

        // TODO: RRT implementation
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
                // Reset position to zero
                //currPoseParams = Eigen::MatrixXd::Zero(1, inputPoses.cols());
                // Set position to previous
                //currPoseParams


                mKrang->setPositions(munzirToDart(currPoseParams.transpose()));
            }

            if (index < inputPoses.rows()) {
                Eigen::MatrixXd finalPoseParams = inputPoses.row(index);
                //40 because we are stepping at 1000 intervals with 25 different
                //pose params (1000/25 = 40)
                int param = timeStep / (40 * scale);
                int step = timeStep % (40 * scale);

                // Below line moves joints one by one
                //currPoseParams.col(param) = finalPoseParams.col(param) * (step + 1) / (40 * scale);
                // Below line moves to straight to final position from before
                // position
                currPoseParams = finalPoseParams;
                mKrang->setPositions(munzirToDart(currPoseParams.transpose()));
            }
            // currPoseParams = inputPoses.row(index);
            // int p = 15;
            // currPoseParams.col(p) = currPoseParams.col(p) + currPoseParams.col(p) * timeStep/100;
            // mKrang->setPositions(currPoseParams.transpose());
            //
            // Eigen::VectorXd q = inputPoses.row(0);
            // q(13) += time * 0.01;
            // q.setZero(); q(14) += time * 0.01;
            // mKrang->setPositions(q);

        }


        SkeletonPtr getKrang() {
            return mKrang;
        }

    protected:
        //Krang
        SkeletonPtr mKrang;
        //Matrix of the poses
        Eigen::MatrixXd inputPoses;
        Eigen::MatrixXd currPoseParams;
};

// // MyWindow
class MyWindow : public SimWindow {
    public:

        // Constructor
        MyWindow(const WorldPtr& world, string robotName, Eigen::MatrixXd inputPoses, CollisionGroupPtr group, CollisionOption& option) : mGroup(group), mOption(option) {
            setWorld(world);
            mController = make_unique<Controller>(
                mWorld->getSkeleton(robotName), inputPoses);

            mInputPoses = inputPoses;
            poseNum = -1;
        }

        void timeStepping() override {
            //Time in milliseconds
            worldTime = (int) (mWorld->getTime()*1000);
            // Use controller to move to the next pose based on the time of
            // simulation
            // Time scale to make simulation seem slower/faster
            // Higher value means slower simulation
            int scale = 2;

            CollisionResult result;
            mGroup->collide(mOption, &result);
            collision = result.isCollision();
            numContacts = result.getNumContacts();

            //poseNum = worldTime / (1000 * scale) + 1;
            cout << "\rWorld Time: " << worldTime << " Pose: " << poseNum + 1 << " Collision: " << collision << " Contacts: " << numContacts << " \t ";

            //mController->setNewPose(worldTime, scale);

            SimWindow::timeStepping();
        }

        // Keyboard input during simulation
        void keyboard(unsigned char _key, int _x, int _y) {
            switch (_key) {
                case 'i': // Print current pose information
                    cout << mController->getKrang()->getPositions().transpose() << "\n";
                    cout << endl;
                    cout << "\rWorld Time: " << worldTime << " Pose: " << poseNum << " Collision: " << collision << " Contacts: " << numContacts << " \t ";
                    break;
                case 'l': // go to next pose
                    if (poseNum + 1 < mInputPoses.rows()) {
                        poseNum++;
                        mController->getKrang()->setPositions(munzirToDart(mInputPoses.row(poseNum)));
                    }
                    cout << "\rWorld Time: " << worldTime << " Pose: " << poseNum + 1 << " Collision: " << collision << " Contacts: " << numContacts << " \t ";
                    break;
                case 'h': // go to previous pose
                    if (poseNum - 1 >= 0) {
                        poseNum--;
                        mController->getKrang()->setPositions(munzirToDart(mInputPoses.row(poseNum)));
                    }
                    cout << "\rWorld Time: " << worldTime << " Pose: " << poseNum  - 1 << " Collision: " << collision << " Contacts: " << numContacts << " \t ";
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
        unique_ptr<Controller> mController;
        string robotName;
        CollisionGroupPtr mGroup;
        CollisionOption& mOption;
        int worldTime;
        int poseNum;
        int collision;
        int numContacts;
        Eigen::MatrixXd mInputPoses;
};

// Function Prototypes
SkeletonPtr createKrang(string fullRobotPath, string robotName);
SkeletonPtr createFloor(string floorName);

// Main Method
int main(int argc, char* argv[]) {
    // INPUT on below line (input pose filename)
    //string inputPosesFilename = "../filteredPosesrandom6003fullbalance0.001000tolsafe2.000000*10e-3filter.txt";
    //string inputPosesFilename = "../poseTrajectoriesfullorderedfinalSet.txt";
    //string inputPosesFilename = "../filteredPosesrandom22106fullbalance0.00100tolsafe";
    //string inputPosesFilename = "../orderedfinalSet.txt";
    string inputPosesFilename = "../finalSetMunzir.txt";
    //string inputPosesFilename = "../poseTrajectoriesrfinalSet/interposeTraj1-2.txt";
    //string inputPosesFilename = "../rfinalSet.txt";
    //string inputPosesFilename = "../random10anglebalance0.001000tolunsafe.txt";

    // INPUT on below line (absolute path of robot)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/KrangVisualCollision.urdf";

    // INPUT on below line (name of robot)
    string robotName = "krang";

    // INPUT on below line (name of floor)
    string floorName = "floor";

    // Read input file
    Eigen::MatrixXd inputPoses;
    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    // create and initialize the world
    WorldPtr world(new World);

    // load skeletons
    SkeletonPtr floor = createFloor(floorName);
    SkeletonPtr mKrang = createKrang(fullRobotPath, robotName);

    mKrang->enableSelfCollisionCheck();
    mKrang->disableAdjacentBodyCheck();

    // Add ground and robot to the world pointer
    world->addSkeleton(floor);
    world->addSkeleton(mKrang);

    // no gravity
    Eigen::Vector3d gravity(0.0, 0.0, 0.0);
    world->setGravity(gravity);

    //TODO
    mKrang->setPositions(munzirToDart(inputPoses.row(0).transpose()));

    // Collision Prep
    auto constraintSolver = world->getConstraintSolver();
    auto group = constraintSolver->getCollisionGroup();
    auto& option = constraintSolver->getCollisionOption();
    auto bodyNodeFilter = std::make_shared<BodyNodeCollisionFilter>();
    option.collisionFilter = bodyNodeFilter;

    CollisionResult result;
    group->collide(option, &result);
    bool collision = result.isCollision();
    double numContacts = result.getNumContacts();

    cout << "Safe Pose Collision: " << collision << endl;

    // Create a window and link it to the world
    MyWindow window(world, robotName, inputPoses, group, option);

    glutInit(&argc, argv);
    window.initWindow(960, 720, "Simulate Poses");
    glutMainLoop();

    cout << endl;
}

SkeletonPtr createKrang(string fullRobotPath, string robotName) {
    // Instantiate krang
    DartLoader loader;
    SkeletonPtr krang = loader.parseSkeleton(fullRobotPath);
    krang->setName(robotName);

    return krang;
}

SkeletonPtr createFloor(string floorName) {

    SkeletonPtr floor = Skeleton::create(floorName);

    // Give the floor a body
    BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    double floor_width = 10;
    double floor_height = 0.05;
    std::shared_ptr<BoxShape> box(
          new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
    auto shapeNode
        = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0 - 0.012);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor;
}

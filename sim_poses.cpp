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
#include <iostream>
#include <fstream>

// Namespaces
using namespace std;
using namespace dart::collision;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::gui;
using namespace dart::math;
using namespace dart::simulation;
using namespace dart::utils;

// Defines
#define MAXBUFSIZE ((int) 1e6)

// Classes
// // Controller
class Controller {
    public:
        /// Constructor
        Controller(const SkeletonPtr& krang, Eigen::MatrixXd poses) : mKrang(krang), inputPoses(poses){
            currPoseParams = Eigen::MatrixXd::Zero(1, inputPoses.cols());
        }

        // TODO:
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
                currPoseParams = Eigen::MatrixXd::Zero(1, inputPoses.cols());

                mKrang->setPositions(currPoseParams.transpose());
            }

            if (index < inputPoses.rows()) {
                Eigen::MatrixXd finalPoseParams = inputPoses.row(index);
                //40 because we are stepping at 1000 intervals with 25 different
                //pose params (1000/25 = 40)
                int param = timeStep / (40 * scale);
                int step = timeStep % (40 * scale);

                currPoseParams.col(param) = finalPoseParams.col(param) * (step + 1) / (40 * scale);
                mKrang->setPositions(currPoseParams.transpose());
            }
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
        MyWindow(const WorldPtr& world, string robotName, Eigen::MatrixXd inputPoses) {
            setWorld(world);
            mController = make_unique<Controller>(
                mWorld->getSkeleton(robotName), inputPoses);
        }

        void timeStepping() override {
            //Time in milliseconds
            int worldTime = (int) (mWorld->getTime()*1000);
            // Use controller to move to the next pose based on the time of
            // simulation
            // Time scale to make simulation seem slower/faster
            // Higher value means slower simulation
            int scale = 5;

            CollisionResult result = mWorld->getLastCollisionResult();
            bool collision = result.isCollision();
            int numContacts = result.getNumContacts();

            int poseNum = worldTime / (1000 * scale) + 1;
            cout << "\rWorld Time: " << worldTime << " Pose: " << poseNum << " Collision: " << collision << " Contacts: " << numContacts;

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
        unique_ptr<Controller> mController;
        string robotName;
};

// Function Prototypes
SkeletonPtr createKrang(string fullRobotPath, string robotName);
SkeletonPtr createFloor(string floorName);

Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename);

// Main Method
int main(int argc, char* argv[]) {
    // INPUT on below line (input pose filename)
    //string inputPosesFilename = "../custom2comfilteredPoses.txt";
    string inputPosesFilename = "../filteredPosesrandomOptPoses100001.000000*10e-3filter.txt";

    // INPUT on below line (absolute path of robot)
    string fullRobotPath = "/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf";

    // INPUT on below line (name of robot)
    string robotName = "krang";

    // INPUT on below line (name of floor)
    string floorName = "floor";

    cout << "Reading Input Poses ...\n";
    Eigen::MatrixXd inputPoses = readInputFileAsMatrix(inputPosesFilename);
    cout << "|-> Done\n";

    // create and initialize the world
    WorldPtr world = World::create();

    // load skeletons
    SkeletonPtr floor = createFloor(floorName);
    SkeletonPtr mKrang = createKrang(fullRobotPath, robotName);

    // A safe pose
    mKrang->setPositions(inputPoses.row(0));
    // An unsafe pose
    mKrang->setPositions(inputPoses.row(7));

    mKrang->enableSelfCollisionCheck();
    mKrang->setAdjacentBodyCheck(true);

    // Add ground and robot to the world pointer
    world->addSkeleton(floor);
    world->addSkeleton(mKrang);

    // no gravity
    Eigen::Vector3d gravity(0.0, 0.0, 0.0);
    world->setGravity(gravity);

    // create a window and link it to the world
    MyWindow window(world, robotName, inputPoses);

    /* Collision Snippet with Dart
    CollisionDetector* detector = world->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);
    bool collision = false;
    size_t collisionCount = detector->getNumContacts();
    for (size_t i = 0; i < collisionCount; ++i) {
        const Contact& contact = detector->getContact(i);
        if (contact.bodyNode1.lock()->getSkeleton() == object || contact.bodyNode2.lock()->getSkeleton() == object) {
            collision = true;
            break;
        }
    }
    if (collision) {
        cout << "Krang in collision!" << endl;
    }
    */

    CollisionResult result = world->getLastCollisionResult();
    Contact contact;
    //CollisionObject colObj1;
    //CollisionObject colObj2;

    for (int i = 0; i < result.getNumContacts(); i++) {
        contact = result.getContact(i);
        CollisionObject colObj1 = contact->collisionObject1;
        //colObj2 = contact->collisionObject2;
    }

    bool collision = result.isCollision();
    cout << collision << endl;

    // TODO
    int numBodies = mKrang->getNumBodyNodes();
    BodyNodePtr bodyi;
    BodyNode* acBodyi = bodyi;

    //CollisionDetector krangColDet();
    //int index = 0;
    //Skeleton skelmKrang = mKrang;
    //ShapeNode krangShapeFrame = skelmKrang.getShapeNode(index);
    //DARTCollisionObject krangColObj(krangColDet, krangShapeFrame);
    //CollisionResult krangColResult();
    //krangColResult.addObject(krangColObj);

    //isColliding = krangColResult.isCollision();

    //for (int i = 0; i < numBodies; i++) {
    //    bodyi = mKrang->getBodyNode(i);

    //    cout << bodyi->getName() << ": ";
    //    //cout << krangColResult.inCollision(acBodyi) << "\n";
    //    //if(acBodyi->isColliding()) {
    //    //for (int j = 0; j < numBodies; j++) {
    //        // is a body always colliding the ones attached to it?
    //        // if so does the pose creation joint limits account for this so
    //        // that we can assume
    //        //if (i != j && bodyi.isColliding()) {

    //              isColliding = krangColResult.isCollision();

    //        //}
    //    //}
    //}
    //cout << isColliding << endl;

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
    return floor;
}

// // Read file as Matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename) {
    // Read numbers (the pose params)
    ifstream infile;
    infile.open(inputPosesFilename);

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    while(! infile.eof()) {
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

    string outShiminArms = "shimin" + inputPosesFilename + ".txt";
    ofstream out_file(outShiminArms);

    // Populate matrix with numbers.
    Eigen::MatrixXd outputMatrix(rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            outputMatrix(i,j) = buff[cols*i+j];
            if (j > 10 && j < 17) {
                out_file << buff[cols*i+j] << ", ";
            } else if (j == 17) {
                out_file << buff[cols*i+j] << endl;
            } else if (j > 17 && j < 24) {
                out_file << buff[cols*i+j] << ", ";
            } else if (j == 24) {
                out_file << buff[cols*i+j] << endl;
            }
        }
    }

    return outputMatrix;
}

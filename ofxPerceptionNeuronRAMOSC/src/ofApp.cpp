#include "ofMain.h"
#include "ofxPerceptionNeuron.h"

#include "ofxOsc.h"
#include "ofxXmlSettings.h"

class ofApp : public ofBaseApp{
    
    ofxPerceptionNeuron::DataReader neuron;
    ofEasyCam cam;
    
    uint16_t dstPort = 10000;
    vector<ofPtr<ofxOscSender>> oscSenders;
    
    string axisNeuronHost = "127.0.0.1";
    int axisNeuronHostPort = 7001;
    
    ofxXmlSettings settings;
    
    bool hideDebugDraw = false;
    bool isFreezed = false;
    
    enum Joint
    {
        JOINT_HIPS              = 0,
        JOINT_ABDOMEN           = 1,
        JOINT_CHEST             = 2,
        JOINT_NECK              = 3,
        JOINT_HEAD              = 4,
        
        JOINT_LEFT_HIP          = 5,
        JOINT_LEFT_KNEE         = 6,
        JOINT_LEFT_ANKLE        = 7,
        JOINT_LEFT_TOE          = 8,
        
        JOINT_RIGHT_HIP         = 9,
        JOINT_RIGHT_KNEE        = 10,
        JOINT_RIGHT_ANKLE       = 11,
        JOINT_RIGHT_TOE         = 12,
        
        JOINT_LEFT_COLLAR       = 13,
        JOINT_LEFT_SHOULDER     = 14,
        JOINT_LEFT_ELBOW        = 15,
        JOINT_LEFT_WRIST        = 16,
        JOINT_LEFT_HAND         = 17,
        
        JOINT_RIGHT_COLLAR      = 18,
        JOINT_RIGHT_SHOULDER    = 19,
        JOINT_RIGHT_ELBOW       = 20,
        JOINT_RIGHT_WRIST       = 21,
        JOINT_RIGHT_HAND        = 22,
        
        NUM_JOINTS              = 23,
    };
    
    string ramJointName[NUM_JOINTS] =
    {
        "HIPS",
        "ABDOMEN",
        "CHEST",
        "NECK",
        "HEAD",
        "LEFT_HIP",
        "LEFT_KNEE",
        "LEFT_ANKLE",
        "LEFT_TOE",
        "RIGHT_HIP",
        "RIGHT_KNEE",
        "RIGHT_ANKLE",
        "RIGHT_TOE",
        "LEFT_COLLAR",
        "LEFT_SHOULDER",
        "LEFT_ELBOW",
        "LEFT_WRIST",
        "LEFT_HAND",
        "RIGHT_COLLAR",
        "RIGHT_SHOULDER",
        "RIGHT_ELBOW",
        "RIGHT_WRIST",
        "RIGHT_HAND"
    };
    
    std::map<int, std::string> pnRamJointMap =
    {
        // key : value
        // RAM Dance Toolkit Joint Name : PerceptionNeuron Bone Name
        {JOINT_HIPS, "Hips"},
        {JOINT_ABDOMEN, "Spine1"},
        {JOINT_CHEST, "Spine3"},
        {JOINT_NECK, "Neck"},
        {JOINT_HEAD, "Head"},
        
        {JOINT_LEFT_HIP, "LeftUpLeg"},
        {JOINT_LEFT_KNEE, "LeftLeg"},
        {JOINT_LEFT_ANKLE, "LeftFoot"},
        {JOINT_LEFT_TOE, "LeftFoot"},     // PN dosn't have toe
        
        {JOINT_RIGHT_HIP, "RightUpLeg"},
        {JOINT_RIGHT_KNEE, "RightLeg"},
        {JOINT_RIGHT_ANKLE, "RightFoot"},
        {JOINT_RIGHT_TOE, "RightFoot"},   // PN dosn't have toe
        
        {JOINT_LEFT_COLLAR, "LeftShoulder"},  // PN dosn't have collar
        {JOINT_LEFT_SHOULDER, "LeftShoulder"},
        {JOINT_LEFT_ELBOW, "LeftArm"},
        {JOINT_LEFT_WRIST, "LeftForeArm"},
        {JOINT_LEFT_HAND, "LeftHand"},
        
        {JOINT_RIGHT_COLLAR, "RightShoulder"},  // PN dosn't have collar
        {JOINT_RIGHT_SHOULDER, "RightShoulder"},
        {JOINT_RIGHT_ELBOW, "RightArm"},
        {JOINT_RIGHT_WRIST, "RightForeArm"},
        {JOINT_RIGHT_HAND, "RightHand"}
    };
    
public:
    void setup()
    {
        #ifdef __APPLE__
            #include "TargetConditionals.h"
            #if TARGET_OS_MAC
                #ifdef DEBUG
                #else
                    ofSetDataPathRoot("../Resources/data/");
                #endif
            #endif
        #endif
        
        if(settings.loadFile("config.xml"))
        {
           axisNeuronHost = settings.getAttribute("axis_neuron", "host", axisNeuronHost);
           axisNeuronHostPort = settings.getAttribute("axis_neuron", "port", axisNeuronHostPort);
           
           int dests = settings.getNumTags("rdtk");
           for(int i=0; i<dests; i++)
           {
               string host = settings.getAttribute("rdtk", "host", "", i);
               int port = settings.getAttribute("rdtk", "port", dstPort, i);
               ofPtr<ofxOscSender> sender = ofPtr<ofxOscSender>(new ofxOscSender());
               sender->setup(host, port);
               oscSenders.push_back(sender);
           }
        }
        ofSetVerticalSync(true);
        ofSetFrameRate(60);
        
        ofSetLogLevel(OF_LOG_VERBOSE);
        
        neuron.connect(axisNeuronHost, axisNeuronHostPort);
    }
    
    void update()
    {
        if(!isFreezed)
        neuron.update();

//        if(neuron.isFrameNew())
        {
            
            vector <ofxPerceptionNeuron::Skeleton> skeletons = neuron.getSkeletons();
            
            for(int i=0; i<skeletons.size(); i++)
            {
                ofxPerceptionNeuron::Skeleton s = skeletons[i];
                
                ofxOscMessage msg;
                msg.setAddress("/ram/skeleton");
                msg.addStringArg(s.getName());
                msg.addIntArg(NUM_JOINTS);
                
                for(int j=0; j<23; j++)
                {
                    ofxPerceptionNeuron::Joint joint = s.getJointByName( pnRamJointMap[j] );
                    ofVec3f v = joint.global_transform.getTranslation();
                    ofQuaternion q = joint.global_transform.getRotate();
                    
                    float a,x,y,z;
                    q.getRotate(a, x, y, z);
                    
                    msg.addStringArg(ramJointName[j]);
                    msg.addFloatArg(v.x);
                    msg.addFloatArg(v.y);
                    msg.addFloatArg(v.z);
                    msg.addFloatArg(a);
                    msg.addFloatArg(x);
                    msg.addFloatArg(y);
                    msg.addFloatArg(z);
                    
                }
                msg.addFloatArg(ofGetElapsedTimef());
                
                for(int h = 0; h < oscSenders.size(); h++)
                {
                    oscSenders[h]->sendMessage(msg);
                }
            }
        }
    }
    
    void draw()
    {
        ofClear(0);
        ofDrawBitmapString("skeltons:"+ofToString(neuron.getSkeletons().size()), 10 , 20);
        cam.begin();
        if (!hideDebugDraw) {
            
            neuron.debugDraw();
        }
        ofDrawAxis(100);
        cam.end();
        
        ofSetWindowTitle(ofToString(ofGetFrameRate()));
    }
    
    void keyPressed(int key)
    {
        if(key == 'h')
            hideDebugDraw = !hideDebugDraw;
        if(key == ' ')
            isFreezed = !isFreezed;
        
    }
    
};

//========================================================================
int main( ){
    ofSetupOpenGL(640, 480, OF_WINDOW);            // <-------- setup the GL context
    
    // this kicks off the running of my app
    // can be OF_WINDOW or OF_FULLSCREEN
    // pass in width and height too:
    ofRunApp(new ofApp());
    
}

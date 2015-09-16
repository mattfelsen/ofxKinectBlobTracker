#include "ofApp.h"

#define kWidth 512
#define kHeight 424
#define COUNTDOWN 0

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();

	blobFinder.init(&kinect, true); // standarized coordinate system: z in the direction of gravity
	blobFinder.setResolution(BF_LOW_RES);
	blobFinder.setRotation(ofVec3f(angle, 0, 0));
	blobFinder.setTranslation(ofVec3f(0, 0, 0));
	blobFinder.setScale(ofVec3f(0.001, 0.001, 0.001)); // mm to meters
													   // bind our kinect to the blob finder
													   // in order to do this we need to declare in ofApp.h: class ofApp : public ofBaseApp, public ofxKinectBlobListener
	blobTracker.setListener(this);

	grayImage.allocate(kWidth, kHeight);
	grayThreshNear.allocate(kWidth, kHeight);
	grayThreshFar.allocate(kWidth, kHeight);
	bgImage.allocate(kWidth, kHeight);
	grayDiff.allocate(kWidth, kHeight);
	grayDiffOfImage.allocate(kWidth, kHeight, OF_IMAGE_GRAYSCALE);

	numPixels = kWidth*kHeight;

	thresholdNear = 0.5;
	thresholdFar = 3;

	// NOTE: measurement units in meters!!!
	minBlobVol = 0.02f;
	maxBlobVol = 2.0f;
	//no cropping
	cropBoxMin = ofVec3f(-10, -10, -10);
	cropBoxMax = ofVec3f(10, 10, 10);
	//
	thresh3D = ofVec3f(0.2, 0.2, 0.3);
	// xy pixel search range
	thresh2D = 1;
	maxBlobs = 10;

	float sqrResolution = blobFinder.getResolution();
	sqrResolution *= sqrResolution;
	minBlobPoints = (int)(0.001*(float)numPixels / sqrResolution);

	printf("min %f\n", minBlobVol);

	bLearnBakground = true;
	timeLapse = 0;
	font.loadFont("PerfectDOSVGA437.ttf", 15, true, true, true);

	ofSetFrameRate(60);

	// start from the front
	bDrawPointCloud = false;
	bDrawIDs = true;
}

//--------------------------------------------------------------
void ofApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if (kinect.getDepthSource()->isFrameNew()) {

		updateDepthImage();

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(depthImg);
		// background subtraction
		if ((bLearnBakground) && (ofGetElapsedTimeMillis() >= timeLapse)) {
			bgImage = grayImage;   // let this frame be the background image from now on
			bLearnBakground = false;
		}
		cvAbsDiff(bgImage.getCvImage(), grayImage.getCvImage(), grayDiff.getCvImage());
		cvErode(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, 2);
		cvDilate(grayDiff.getCvImage(), grayDiff.getCvImage(), NULL, 1);
		// threshold ignoring little differences
		cvThreshold(grayDiff.getCvImage(), grayDiff.getCvImage(), 3, 255, CV_THRESH_BINARY);
		grayDiff.flagImageChanged();
		// update the ofImage to be used as background mask for the blob finder
		grayDiffOfImage.setFromPixels(grayDiff.getPixels(), kWidth, kHeight, OF_IMAGE_GRAYSCALE);

		blobFinder.findBlobs(&grayDiffOfImage,
			cropBoxMin, cropBoxMax,
			thresh3D, thresh2D,
			minBlobVol, maxBlobVol, minBlobPoints, maxBlobs);
		blobTracker.trackBlobs(blobFinder.blobs);
	}

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {

	ofSetColor(255, 255, 255);

	if (bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	}
	else {
		// draw from the live kinect
		depthImg.draw(10, 10, 320, 240);
		kinect.getColorSource()->getTexture().draw(340, 10, 320, 240);

		grayImage.draw(10, 260, 320, 240);

		grayDiff.draw(340, 260, 320, 240);

	}

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "press b to set current frame as background" << endl
		<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
		<< "num blobs found " << blobFinder.nBlobs
		<< ", fps: " << ofGetFrameRate() << endl
		<< "press i to toggle draw blob IDs in 3D scene" << endl
		<< "press c to close the connection and o to open it again, connection is: " << kinect.isOpen() << endl
		<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(), 20, 510);

	char timeLapseStr[256];
	if (bLearnBakground) {
		ofSetHexColor(0xffffff);
		sprintf(timeLapseStr, "%i", timeLapse - ofGetElapsedTimeMillis());
		ofPushMatrix();
		font.drawString(timeLapseStr, ofGetWidth() / 2, (ofGetHeight() - font.getLineHeight()) / 2);
		ofPopMatrix();
	}
}

//--------------------------------------------------------------
void ofApp::updateDepthImage() {
	if (!depthImg.isAllocated()) {
		depthImg.allocate(kWidth, kHeight, OF_IMAGE_GRAYSCALE);
	}

	ofShortPixels pix;
	pix.allocate(kWidth, 1, OF_IMAGE_GRAYSCALE);

	auto& depthPix = kinect.getDepthSource()->getPixels();
	for (int y = 0; y < kHeight; y++) {

		for (int x = 0; x < kHeight; x++) {
			int index = x + (y*kWidth);

			// Build the 8bit, thresholded image for drawing to screen
			if (depthPix.getWidth() && depthPix.getHeight()) {
				// Multiply thresholds by 1000 because the values are in meters in world
				// space but in mm in the depthPix array
				float depth = depthPix[index];
				float val = depth == 0 ? 0 : ofMap(depth, thresholdNear * 1000, thresholdFar * 1000, 255, 0, true);
				depthImg.setColor(x, y, ofColor(val));
			}
		}

	}

	depthImg.update();
}

//--------------------------------------------------------------
void ofApp::drawPointCloud() {
	ofPushMatrix();
	ofScale(100.0, 100.0, 100.0);
	glEnable(GL_DEPTH_TEST);
	glPointSize(3);
	// draw blobs
	for (unsigned int i = 0; i < blobFinder.blobs.size(); i++) {
		ofSetColor(25 * i, 25 * i, 255 - 25 * i);
		// draw blobs
		blobFinder.blobs[i].draw();
		// plot blobs IDs
		if (bDrawIDs) {
			ofPushMatrix();
			ofTranslate(blobTracker.blobs[i].massCenter.x, blobTracker.blobs[i].massCenter.y, blobTracker.blobs[i].maxZ.z);
			ofRotateX(-90);
			ofScale(0.01f, 0.01f, 0.01f);
			ofSetColor(255, 255, 255);
			font.drawStringAsShapes(ofToString(blobTracker.blobs[i].id), 0, 0);
			ofPopMatrix();
		}
		// draw trajectory as a line
		vector <ofVec3f> trajectory;
		blobTracker.getTrajectoryById(blobTracker.blobs[i].id, trajectory);
		unsigned int trjSize = trajectory.size();
		if (trjSize > 1) {
			ofPushMatrix();
			ofSetColor(255, 255, 0);
			ofSetLineWidth(3);
			glBegin(GL_LINE);
			for (unsigned int j = 0; j < trjSize; j++) {
				glVertex3f(trajectory[j].x, trajectory[j].y, trajectory[j].z);
			}
			glEnd();
			ofPopMatrix();
			trajectory.clear();
		}
	}
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}
//--------------------------------------------------------------
void ofApp::exit() {
	kinect.close();

#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {

	case 'b':
		bLearnBakground = true;
		// to give you time to run away of the field of view
		timeLapse = COUNTDOWN + ofGetElapsedTimeMillis();
		break;

	case'p':
		bDrawPointCloud = !bDrawPointCloud;
		break;

	case'i':
		bDrawIDs = !bDrawIDs;
		break;

	case OF_KEY_UP:
		if (ofGetKeyPressed(OF_KEY_SHIFT)) thresholdFar += 0.1;
		else thresholdNear += 0.1;
		break;

	case OF_KEY_DOWN:
		if (ofGetKeyPressed(OF_KEY_SHIFT)) thresholdFar -= 0.1;
		else thresholdNear -= 0.1;
		break;

	/*
	case 'w':
		kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		break;
	*/

	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
/*
*
*	blob section
*
*	from here on in it's blobs
*	thanks to stefanix and the opencv library :)
*
*/

//--------------------------------------------------
void ofApp::blobOn(ofVec3f centroid, int id, int order) {
	// cout << "blobOn() - id:" << id << " order:" << order << endl;
}

void ofApp::blobMoved(ofVec3f centroid, int id, int order) {
	//  cout << "blobMoved() - id:" << id << " order:" << order << endl;
	// full access to blob object ( get a reference)
	//  ofxKinectTrackedBlob blob = blobTracker.getById( id );
	// cout << "volume: " << blob.volume << endl;
}

void ofApp::blobOff(ofVec3f centroid, int id, int order) {
	// cout << "blobOff() - id:" << id << " order:" << order << endl;
}

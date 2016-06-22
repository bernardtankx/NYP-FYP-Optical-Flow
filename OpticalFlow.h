//---------------------------------------------------------------------------
#ifndef OpticalFlowH
#define OpticalFlowH
#include <vcl.h>
#include "Globals.h"
#include "Geometry.h"
#include "lazyopencv\\lazyopencv110.h"
//---------------------------------------------------------------------------
class OpticalFlow {
   int width, height;
   int halfWidth, halfHeight;
   CvSize imageSize, pyramidSize;
   CvPoint2D32f *cornersA, *cornersB;
   IplImage *pyramidA, *pyramidB;
   int cornerCount, foundCornerCount, MAX_CORNERS;
   IplImage *image[ 2 ];
   IplImage *eigImage;
   IplImage *tmpImage;
   int curr;
   char featuresFound[ 1000 ];
   float featureErrors[ 1000 ];
   float rollAngle[ 2 ], pitchAngle[ 2 ];
   float distanceX, distanceY, angle;
   float viewAngleX, halfViewAngleX;
   float viewAngleY, halfViewAngleY;
   float newAltitude, newAngularVelocity, x, y;
   float cx, cy;
   
   float prevCentroidX, prevCentroidY, currCentroidX, currCentroidY;
   float prevXm[ 1000 ], prevYm[ 1000 ];
   float currXm[ 1000 ], currYm[ 1000 ];
   float prevCentroidPixelX, prevCentroidPixelY, currCentroidPixelX, currCentroidPixelY;
   float prevPixelX[ 1000 ], prevPixelY[ 1000 ];
   float currPixelX[ 1000 ], currPixelY[ 1000 ];
   float aMean;
public:
   float kappa;
   OpticalFlow();
   OpticalFlow( int width, int height );
   ~OpticalFlow();
   void update( float T, Geometry *geometry, unsigned char *rgbImage,
                float rollAngle, float pitchAngle, float altitude );
   void getVelocity( float& velocityX, float& velocityY );
	float getAngularVelocity();
	float getAltitude();
	void undistort( float x, float y, float& nx, float& ny );
	void drawVectors( TPaintBox *paintBox );
};
//---------------------------------------------------------------------------
#endif

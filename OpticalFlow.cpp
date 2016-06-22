//---------------------------------------------------------------------------
#include "OpticalFlow.h"
#include "UnitMain.h"
//---------------------------------------------------------------------------
OpticalFlow::OpticalFlow()
{
}
//---------------------------------------------------------------------------
OpticalFlow::OpticalFlow( int width, int height )
{
   this->width = width;
   this->height = height;
   halfWidth = width / 2;
   halfHeight = height / 2;

   imageSize = cvSize( width, height );

   // create 2 gray images
   for( int i = 0; i < 2; i++ ) {
      image[ i ] = cvCreateImage( imageSize, 8, 1 );
   }

   MAX_CORNERS = 5;

   // structures to store corners
   cornersA = new CvPoint2D32f[ MAX_CORNERS ];
   cornersB = new CvPoint2D32f[ MAX_CORNERS ];
   cornerCount = MAX_CORNERS;

   eigImage = cvCreateImage( imageSize, IPL_DEPTH_32F, 1 );
   tmpImage = cvCreateImage( imageSize, IPL_DEPTH_32F, 1 );

   pyramidSize = cvSize( width + 8, height / 3 );
   pyramidA = cvCreateImage( pyramidSize, IPL_DEPTH_32F, 1 );
   pyramidB = cvCreateImage( pyramidSize, IPL_DEPTH_32F, 1 );

   curr = 0;

/*
	float xx = ( width - 1 ) / 11.0;
	float yy = ( height - 1 ) / 10.0;
	cornerCount = 0;
	for( int j = 0; j < 3; j++ ) {
		for( int i = 0; i < 4; i++ ) {
			( cornersA + cornerCount )->x = 4.0 * xx + i * xx;
			( cornersA + cornerCount )->y = 4.0 * yy + j * yy;
			cornerCount++;
		}
	}

	float xx = ( width - 1 ) / 17.0;
	float yy = ( height - 1 ) / 16.0;
	cornerCount = 0;
	for( int j = 0; j < 3; j++ ) {
		for( int i = 0; i < 4; i++ ) {
			( cornersA + cornerCount )->x = 7.0 * xx + i * xx;
			( cornersA + cornerCount )->y = 7.0 * yy + j * yy;
			cornerCount++;
		}
	}
*/

   viewAngleX = Globals::viewAngleX;
   halfViewAngleX = viewAngleX / 2.0;
   viewAngleY = Globals::viewAngleY;
   halfViewAngleY = viewAngleY / 2.0;

   pitchAngle[ 0 ] = 0.0;
   pitchAngle[ 1 ] = 0.0;

   kappa = 0.00284;

   cx = ( width - 1 ) / 2.0;
   cy = ( height - 1 ) / 2.0;
}
//---------------------------------------------------------------------------
OpticalFlow::~OpticalFlow()
{
}
//---------------------------------------------------------------------------
void OpticalFlow::update( float T, Geometry *geometry, unsigned char *rgbImage,
                          float rollAngle, float pitchAngle, float altitude )
{
   float ax, ay, bx, by;
   unsigned char *src, *dst;

   src = rgbImage;
   dst = image[ curr ]->imageData;

   // calculate grayscale image
   for( int j = 0; j < height; j++ ) {
      for( int i = 0; i < width; i++ ) {
         *dst = ( *src + *( src + 1 ) + *( src + 2 ) ) / 3;
         src += 4;
         dst++;
      }
   }

   this->rollAngle[ curr ] = rollAngle;
   this->pitchAngle[ curr ] = pitchAngle;

	// find distinctive points in the image
	// cornerMinEigenVal ========================================================
	//cvGoodFeaturesToTrack( image[ curr ], eigImage, tmpImage, cornersA, &cornerCount,
	//							  0.00000000005, 0.001, 0, 3, 0, 0.04  );
	// cornerHarris =============================================================
	cvGoodFeaturesToTrack( image[ curr ], eigImage, tmpImage, cornersA, &cornerCount,
								  0.00000000005, 0.001, 0, 3, 1, 0.04  );

	cvCalcOpticalFlowPyrLK( image[ 1 - curr ], image[ curr ], pyramidA, pyramidB,
                           cornersA, cornersB, cornerCount,
                           cvSize( 15, 15 ), 5, featuresFound, featureErrors,
                           cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
                           20, 0.3 ), 0 );

   // calculate velocity in x- and y- directions
   distanceX = 0.0;
   distanceY = 0.0;
   foundCornerCount = 0;

   prevCentroidX = 0.0;
   prevCentroidY = 0.0;
   currCentroidX = 0.0;
   currCentroidY = 0.0;
   prevCentroidPixelX = 0.0;
   prevCentroidPixelY = 0.0;
   currCentroidPixelX = 0.0;
   currCentroidPixelY = 0.0;

   for( int i = 0; i < cornerCount; i++ ) {
      if( ( featuresFound[ i ] == 0 ) || ( featureErrors[ i ] > 550 ) ) continue;
      undistort( cornersA[ i ].x, cornersA[ i ].y, ax, ay );
      undistort( cornersB[ i ].x, cornersB[ i ].y, bx, by );

      // compensate for roll angle
      float currX = geometry->getDistanceOnFloor( width, viewAngleX, this->rollAngle[ curr ], altitude, halfWidth - bx );
      float prevX = geometry->getDistanceOnFloor( width, viewAngleX, this->rollAngle[ 1 - curr ], altitude, halfWidth - ax );

      // compensate for pitch angle
      float currY = geometry->getDistanceOnFloor( height, viewAngleY, this->pitchAngle[ curr ], altitude, halfHeight - by );
      float prevY = geometry->getDistanceOnFloor( height, viewAngleY, this->pitchAngle[ 1 - curr ], altitude, halfHeight - ay );

      distanceX += ( currX - prevX );
      distanceY += ( currY - prevY );

      prevXm[ foundCornerCount ] = prevX;
      prevYm[ foundCornerCount ] = prevY;
      currXm[ foundCornerCount ] = currX;
      currYm[ foundCornerCount ] = currY;
      prevPixelX[ foundCornerCount ] = ax;
      prevPixelY[ foundCornerCount ] = ay;
      currPixelX[ foundCornerCount ] = bx;
      currPixelY[ foundCornerCount ] = by;

      prevCentroidX += prevX;
      prevCentroidY += prevY;
      currCentroidX += currX;
      currCentroidY += currY;
      prevCentroidPixelX += ax;
      prevCentroidPixelY += ay;
      currCentroidPixelX += bx;
      currCentroidPixelY += by;

      foundCornerCount++;
   }

   if( foundCornerCount > 0 ) {
      distanceX /= ( float )foundCornerCount;
      distanceY /= ( float )foundCornerCount;
      prevCentroidX /= ( float )foundCornerCount;
      prevCentroidY /= ( float )foundCornerCount;
      currCentroidX /= ( float )foundCornerCount;
      currCentroidY /= ( float )foundCornerCount;
      prevCentroidPixelX /= ( float )foundCornerCount;
      prevCentroidPixelY /= ( float )foundCornerCount;
		currCentroidPixelX /= ( float )foundCornerCount;
		currCentroidPixelY /= ( float )foundCornerCount;
	}
/*
	// Angular Velocity =========================================================
	float aPrev, aCurr, a;
	angle = 0.0;

	for( int i = 0; i < foundCornerCount; i++ ) {
		float dx = prevXm[ i ] - prevCentroidX;
		float dy = prevYm[ i ] - prevCentroidY;
		aPrev = 0.0;
		if( ( dx != 0.0 ) && ( dy != 0.0 ) ) {
			aPrev = atan2( dy, dx ) * 180.0 / M_PI;
		}

		dx = currXm[ i ] - currCentroidX;
		dy = currYm[ i ] - currCentroidY;
		aCurr = 0.0;
		if( ( dx != 0.0 ) && ( dy != 0.0 ) ) {
			aCurr = atan2( dy, dx ) * 180.0 / M_PI;
		}
		a = aCurr - aPrev;
		if( a > 180.0 ) a -= 360.0;
		if( a < -180.0 ) a += 360.0;

		angle += a;
	}

	if( foundCornerCount > 0 ) {
		angle /= ( float )foundCornerCount;
	}

	newAngularVelocity = ( angle / T );

	// Altitude =================================================================
	float rPrev, rCurr, rRatioTotal, altitudeRatio, uncountedCorner;

	altitudeRatio = 1.0;
	rRatioTotal = 0.0;
	uncountedCorner = 0.0;

	if( ( newAngularVelocity > -1.5 ) && ( newAngularVelocity < 1.5 ) ) {
		for( int k = 0; k < foundCornerCount; k++ ) {
			float ex = prevPixelX[ k ] - prevCentroidPixelX;
			float ey = prevPixelY[ k ] - prevCentroidPixelY;

			rPrev = sqrt( ( ex * ex ) + ( ey * ey ) );

			ex = currPixelX[ k ] - currCentroidPixelX;
			ey = currPixelY[ k ] - currCentroidPixelY;

			rCurr = sqrt( ( ex * ex ) + ( ey * ey ) );

			if( rCurr != 0.0 ) {
				rRatioTotal += ( rPrev / rCurr );
			} else {
				uncountedCorner++;
			}
		}

		if( ( foundCornerCount - uncountedCorner ) > 0 ) {
			altitudeRatio = rRatioTotal / ( ( float )foundCornerCount - uncountedCorner );
		}
	}
	newAltitude = altitude + ( altitudeRatio - 1.0 ) * altitude;
*/
	// Get PositionX and PositionY
   x = -distanceX / T;
   y = distanceY / T;

	curr = 1 - curr;
}
//---------------------------------------------------------------------------
void OpticalFlow::undistort( float x, float y, float& nx, float& ny )
{
   float k;

   float dx = x - cx;
   float dy = y - cy;
   float d = sqrt( dx * dx + dy * dy );
   float q = 4.0 * kappa * d;
   k = 1.0;
   if( q < 1.0 ) {
      float u = ( 1.0 - sqrt( 1.0 - q ) ) / 2.0 / kappa;
      k = u / d;
   }
   nx = x + k * dx;
   ny = y + k * dy;
}
//---------------------------------------------------------------------------
void OpticalFlow::getVelocity( float& velocityX, float& velocityY )
{
   velocityX = x;
   velocityY = y;
}
//---------------------------------------------------------------------------
float OpticalFlow::getAngularVelocity()
{
	return( newAngularVelocity );
}
//---------------------------------------------------------------------------
float OpticalFlow::getAltitude()
{
	return( newAltitude );
}
//---------------------------------------------------------------------------
void OpticalFlow::drawVectors( TPaintBox *paintBox )
{
/*
	for( int i = 0; i < cornerCount; i++ ) {
		if( ( featuresFound[ i ] == 0 ) || ( featureErrors[ i ] > 550 ) ) continue;
		paintBox->Canvas->MoveTo( cornersA[ i ].x, height - cornersA[ i ].y - 1 );
		paintBox->Canvas->LineTo( cornersB[ i ].x, height - cornersB[ i ].y - 1 );
	}
*/

	float ax, ay, bx, by;
	for( int i = 0; i < cornerCount; i++ ) {
		if( ( featuresFound[ i ] == 0 ) || ( featureErrors[ i ] > 550 ) ) continue;
		ax = cornersA[ i ].x;
		ay = cornersA[ i ].y;

		paintBox->Canvas->MoveTo( ax, height - ay - 1 );
		paintBox->Canvas->Ellipse( ax - 2, height - ay - 2, ax + 4, height - ay + 4 );
	}

//   paintBox->Canvas->Pen->Color = clRed;
//   paintBox->Canvas->MoveTo( 80, 60 );
//   paintBox->Canvas->LineTo( 80 + sumX, 60 - sumY );
}
//---------------------------------------------------------------------------


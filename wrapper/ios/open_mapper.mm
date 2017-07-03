// (c) 2017 OpenMapper

#include "open_mapper.h"

#include <fstream>
#include <iostream>
#include <functional>

#include "opencv2/imgcodecs/ios.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "openmapper/openmapper.h"


struct OpenMapperOpaqueMembers {
  openmapper::OpenMapper open_mapper;
};

@implementation OpenMapper

int counter = 0;

-(id) init {
  self = [super init];
  if (self) {
//    openmapper_members_ = new OpenMapperOpaqueMembers();
  }
  return self;
}

- (void) dealloc {
  // Free members
  delete openmapper_members_;
}

-(void) setDefaultFbo: (unsigned int) default_fbo {
  // pass default FBO to openGL 
}

-(void) processUIImage:(UIImage *) camera_image {
  cv::Mat image;
  UIImageToMat(camera_image, image, false);
    
}

-(void) draw {
  // draw opengl stuff
}

@end

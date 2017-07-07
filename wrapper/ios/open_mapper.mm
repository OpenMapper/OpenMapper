// (c) 2017 OpenMapper

// needs to be included before any apple header due to conflicts of macros
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/blenders.hpp"

#include "open_mapper.h"

#include <fstream>
#include <iostream>
#include <functional>

#include "opencv2/imgcodecs/ios.h"
//#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "openmapper/openmapper.h"


struct OpenMapperOpaqueMembers {
  OpenMapperOpaqueMembers(const std::vector<std::string>& flags) : open_mapper(flags) {}
  openmapper::OpenMapper open_mapper;
};

@implementation OpenMapper

int counter = 0;

-(id) init {
  self = [super init];
  if (self) {
    
    std::vector<std::string> flags;
    
    NSString *path_to_vocabulary_ns = [[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"ORBvoc.txt"];
    std::string path_to_vocabulary = std::string([path_to_vocabulary_ns UTF8String]);
    flags.push_back(path_to_vocabulary);
    NSString *path_to_config_ns = [[[NSBundle mainBundle] resourcePath] stringByAppendingPathComponent:@"iphone.yaml"];
    std::string path_to_config = std::string([path_to_config_ns UTF8String]);
    flags.push_back(path_to_config);
    openmapper_members_ = new OpenMapperOpaqueMembers(flags);
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

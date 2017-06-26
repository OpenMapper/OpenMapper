// (c) 2017 OpenMapper

#ifndef OPEN_MAPPER_H
#define OPEN_MAPPER_H

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>
#import <CoreMedia/CoreMedia.h>

@interface OpenMapper : NSObject {
  // opaque pointer to hide cpp members from header and swift
  struct OpenMapperOpaqueMembers *openmapper_members_;
}

-(id)init;
-(void)dealloc;
-(void) processUIImage:(UIImage *) camera_image;
-(void) draw;
-(void) setDefaultFbo: (unsigned int) default_fbo;

@end
#endif // OPEN_MAPPER_H

mkdir -p fw_build
cd fw_build
cmake ../. -DBUILD_IOS_FW=TRUE -GXcode

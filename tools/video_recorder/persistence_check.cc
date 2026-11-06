// Check the external encoder API contract: saved video survives destruction.
#include <gz/common/VideoEncoder.hh>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

int main(int argc,char **argv){
  if(argc!=2)return 2;
  std::string output=argv[1],temporary=output+".temporary.mp4";
  {
    gz::common::VideoEncoder encoder;
    if(!encoder.Start("mp4",temporary,320,180,25,500000))return 3;
    std::vector<unsigned char> pixels(320*180*3,100);
    for(int i=0;i<10;++i){
      if(!encoder.AddFrame(pixels.data(),320,180,
          std::chrono::steady_clock::time_point(std::chrono::milliseconds(i*40))))return 4;
    }
    if(!encoder.SaveToFile(output))return 5;
  }
  if(!std::filesystem::exists(output)||std::filesystem::file_size(output)<100)return 6;
  if(std::filesystem::exists(temporary))return 7;
  std::cout<<"PASS: saved MP4 survives encoder destruction\n";
}

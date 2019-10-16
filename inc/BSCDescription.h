//
// Created by Jianping on 2019/10/16.
//

#ifndef BASICNEIGHBORHOODFEATURES_BSCDESCRIPTION_H
#define BASICNEIGHBORHOODFEATURES_BSCDESCRIPTION_H

#include <vector>

namespace BNF{

class BSCDescriptor{
public:
    BSCDescriptor(int size):
    _size(size),_bytes(size/8)
    {
        _feature.resize(_bytes);
        memset(_feature.data(),0,sizeof(char)*_bytes);
    }

    BSCDescriptor(const std::vector<char>& feature){
        _size = feature.size()*8;
        _bytes = feature.size();
        _feature.resize(_bytes);
        memcpy(_feature.data(),feature.data(), sizeof(char)*_bytes);
    }

    int hammingDistance(const BSCDescriptor& bsdf1, const BSCDescriptor& bsdf2);

    std::vector<char> getFeature();

private:
    unsigned char byteBitsLookUp(unsigned char b);
private:
    int _size;
    int _bytes;
    std::vector<char> _feature;
};

inline std::vector<char> BSCDescriptor::getFeature() {
    return _feature;
}

}

#endif //BASICNEIGHBORHOODFEATURES_BSCDESCRIPTION_H

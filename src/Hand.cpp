#include <Hand.h>

Hand::Hand() {
    Hand::Init();
}

void Hand::Init() {
    //nothing to do
    if (Hand::CalibratePosition()) {
    }
}

bool Hand::CalibratePosition() {
    return true;
}
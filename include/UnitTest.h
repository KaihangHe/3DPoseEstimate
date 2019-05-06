//
// Created by nicapoet on 19-3-3.
//

#ifndef SFM_UNITTEST_H
#define SFM_UNITTEST_H

#include "Calibrator.h"
#include "FeaturePtsCatcher.h"
#include "CloudPtsGenerator.h"
#include "MindVisionCamera.h"

class UnitTest {
public:
    void Calibrator_test();

    void Cloud_test();

    void Feature_Points_Match_test();

    void multipates_camera_test();
};


#endif //SFM_UNITTEST_H

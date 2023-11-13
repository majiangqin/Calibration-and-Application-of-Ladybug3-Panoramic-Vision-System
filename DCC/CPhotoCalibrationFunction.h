#ifndef PHOTO_CALIBRATION_FUNCTION_H
#define PHOTO_CALIBRATION_FUNCTION_H

#include "CFunction.h"
#include "PicData.h"

#pragma once

// This general CPhotoCalibration includes only Xs, Ys, Zs, ¦Õ,¦Ø,¦Ê
class CPhotoCalibration : public CFunction
{
public:
   CPhotoCalibration(const PicData& picData, const ObjectData& objectData, 
      const SimplexVertex& distortParams, const ImagePoint& origin, double f);
   virtual ~CPhotoCalibration() {};

   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

   virtual Matrix Jacobian(const SimplexVertex& vx) const;

private:
   PicData m_picData;
   ObjectData m_objectData;
   SimplexVertex m_distortParams;
   ImagePoint m_origin;
   double m_f;
};

// This function class includes all internal and external parameters in photo calibration
class CPhotoCalibrationAll : public CFunction
{
public:
   CPhotoCalibrationAll(const PicData& picData, const ObjectData& objectData);
   virtual ~CPhotoCalibrationAll() {};

   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

   virtual Matrix Jacobian(const SimplexVertex& vx) const;

private:
   PicData m_picData;
   ObjectData m_objectData;
};

// This function class doesn't include distortion parameters in photo calibration
class CPhotoCalibrationWithoutDistort : public CFunction
{
public:
   CPhotoCalibrationWithoutDistort(const PicData& picData, const ObjectData& objectData);
   virtual ~CPhotoCalibrationWithoutDistort() {};

   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

   virtual Matrix Jacobian(const SimplexVertex& vx) const;

private:
   PicData m_picData;
   ObjectData m_objectData;
};

// This function class initiates external elements of camera
class CPhotoCalibrationInitXYZFunc : public CFunction
{
public:
   CPhotoCalibrationInitXYZFunc(const PicData& picData, const ObjectData& objectData, 
      const SimplexVertex& distortParams, const ImagePoint& origin, double f);
   virtual ~CPhotoCalibrationInitXYZFunc() {};

   virtual SimplexVertex operator()(const SimplexVertex& vx) const;

   virtual Matrix Jacobian(const SimplexVertex&) const;

private:
   PicData           m_picData;
   ObjectData        m_objectData;
   SimplexVertex     m_distortParams;
   ImagePoint        m_origin;
   double            m_f;
};
#endif
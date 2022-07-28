/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        //ar & mspKeyFrames;
        //ar & mspMapPoints;
        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();                 // Construct an empty map
    Map(int initKFid);     // Construct with initial KF id —— set mnInitKFid = initKFid, mnMaxKFid = initKFid
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);        // mspKeyFrames.insert(pKF)
    void AddMapPoint(MapPoint* pMP);        // mspMapPoints.insert(pMP)
    void EraseMapPoint(MapPoint* pMP);      // mspMapPoints.erase(pMP);
    void EraseKeyFrame(KeyFrame* pKF);      // mspKeyFrames.erase(pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs); //  mvpReferenceMapPoints = vpMPs;
    void InformNewBigChange();              // mnBigChangeIdx++
    int GetLastBigChangeIdx();              // return mnBigChangeIdx

    std::vector<KeyFrame*> GetAllKeyFrames();               // return mspKeyFrames 
    std::vector<MapPoint*> GetAllMapPoints();               // return mspMapPoints
    std::vector<MapPoint*> GetReferenceMapPoints();         // mvpReferenceMapPoints

    long unsigned int MapPointsInMap();                     // return mspMapPoints.size()
    long unsigned  KeyFramesInMap();                        // return mspKeyFrames.size()

    long unsigned int GetId();                              // return mnId;

    long unsigned int GetInitKFid();                        // mnInitKFid
    void SetInitKFid(long unsigned int initKFif);           // mnInitKFid = initKFif;
    long unsigned int GetMaxKFid();                         // return mnMaxKFid

    KeyFrame* GetOriginKF();    // mpKFinitial

    void SetCurrentMap();       // mIsInUse = true;
    void SetStoredMap();        // mIsInUse = false;

    bool HasThumbnail();        // Cann't find the implementaion !
    bool IsInUse();             // return mIsInUse;

    void SetBad();              // mbBad = true
    bool IsBad();               // return mbBad

    void clear();

    int GetMapChangeIndex();                         // return mnMapChange;
    void IncreaseChangeIndex();                      // mnMapChange++
    int GetLastMapChange();                          // return mnMapChangeNotified;
    void SetLastMapChange(int currentChangeId);      // mnMapChangeNotified = currentChangeId;

    void SetImuInitialized();
    bool isImuInitialized();

    // Set body (IMU) coord of first keyframe is as the world coord, its position is (0, 0, 0)
    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams);

    // Where is the implementation of this function ?
    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:

    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;   // All MapPoints in this map - Important!
    std::set<KeyFrame*> mspKeyFrames;   // All KeyFrames in this map - Important!

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;  // Initail keyframe
    KeyFrame* mpKFlowerID;  // The keyframe with minimum id

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;    // Initial keyframe id
    long unsigned int mnMaxKFid;     // Maximum id of all inserted keyframes
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;

};

} //namespace ORB_SLAM3

#endif // MAP_H

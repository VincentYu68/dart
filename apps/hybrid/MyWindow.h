#ifndef _MYWINDOW_
#define _MYWINDOW_

#include <stdarg.h>
#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "dynamics/SkeletonDynamics.h"

class Controller;

class MyWindow : public dart::yui::Win3D, public dart::integration::IntegrableSystem {
public:
 MyWindow(dart::dynamics::SkeletonDynamics* _mList = 0, ...): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mSim = false;
        mPlay = false;
        mSimFrame = 0;
        mPlayFrame = 0;

        mPersp = 30.f;
        mTrans[2] = -1.f;
    
        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/1000.0;
        mForce = Eigen::Vector3d::Zero();

        if (_mList) {
            mSkels.push_back(_mList);
            va_list ap;
            va_start(ap, _mList);
            while (true) {
                dart::dynamics::SkeletonDynamics *skel = va_arg(ap, dart::dynamics::SkeletonDynamics*);
                if(skel)
                    mSkels.push_back(skel);
                else
                    break;
            }
            va_end(ap);
        }
        
        int sumNDofs = 0;
        mIndices.push_back(sumNDofs);
        for (unsigned int i = 0; i < mSkels.size(); i++) {
            int nDofs = mSkels[i]->getNumDofs();
            sumNDofs += nDofs;
            mIndices.push_back(sumNDofs);
        }
        initDyn();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(const Eigen::VectorXd &state);	

 protected:	
    int mSimFrame;
    bool mSim;
    int mPlayFrame;
    bool mPlay;
    dart::integration::EulerIntegrator mIntegrator;
    std::vector<Eigen::VectorXd> mBakedStates;

    std::vector<dart::dynamics::SkeletonDynamics*> mSkels;
    std::vector<Eigen::VectorXd> mDofVels;
    std::vector<Eigen::VectorXd> mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    Eigen::Vector3d mForce;
    std::vector<int> mIndices;
    Controller *mController;

    void initDyn();
    void setPose();
    void bake();
};

#endif

// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// LGPL License

#ifndef KDL_CHAIN_IKSOLVERVEL_PINV_HPP
#define KDL_CHAIN_IKSOLVERVEL_PINV_HPP

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "svd_HH.hpp"

namespace KDL
{
    class ChainIkSolverVel_pinv : public ChainIkSolverVel
    {
    public:
        explicit ChainIkSolverVel_pinv(const Chain& chain, double eps = 0.00001, int maxiter = 150);
        ~ChainIkSolverVel_pinv();

        virtual int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);
        virtual int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out) { return -1; };

        virtual void updateInternalDataStructures() {}

    private:
        const Chain chain;
        ChainJntToJacSolver jnt2jac;
        Jacobian jac;
        SVD_HH svd;
        std::vector<JntArray> U;
        JntArray S;
        std::vector<JntArray> V;
        JntArray tmp;
        double eps;
        int maxiter;
    };
}
#endif

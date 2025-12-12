// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// LGPL License

#include "chainiksolvervel_pinv.hpp"
#include <cmath>

namespace KDL
{
    ChainIkSolverVel_pinv::ChainIkSolverVel_pinv(const Chain& _chain, double _eps, int _maxiter) :
        chain(_chain),
        jnt2jac(chain),
        jac(chain.getNrOfJoints()),
        svd(jac),
        U(6, JntArray(chain.getNrOfJoints())),
        S(chain.getNrOfJoints()),
        V(chain.getNrOfJoints(), JntArray(chain.getNrOfJoints())),
        tmp(chain.getNrOfJoints()),
        eps(_eps),
        maxiter(_maxiter)
    {
    }

    ChainIkSolverVel_pinv::~ChainIkSolverVel_pinv() {}

    int ChainIkSolverVel_pinv::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
    {
        double sum;
        unsigned int i, j;

        jnt2jac.JntToJac(q_in, jac);
        for (i = 0; i < jac.columns(); i++) {
            for (j = 3; j < jac.rows(); j++) {
                jac(j, i) = 0;
            }
        }

        int ret = svd.calculate(jac, U, S, V, maxiter);

        for (i = 0; i < jac.columns(); i++) {
            sum = 0.0;
            for (j = 0; j < jac.rows(); j++) {
                sum += U[j](i) * v_in(j);
            }
            tmp(i) = sum * (std::fabs(S(i)) < eps ? 0.0 : 1.0 / S(i));
        }

        for (i = 0; i < jac.columns(); i++) {
            sum = 0.0;
            for (j = 0; j < jac.columns(); j++) {
                sum += V[i](j) * tmp(j);
            }
            qdot_out(i) = sum;
        }
        return ret;
    }
}

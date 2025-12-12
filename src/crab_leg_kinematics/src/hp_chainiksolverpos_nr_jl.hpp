// Copyright  (C)  2007-2008  Ruben Smits, Mikael Mayer, Julia Jesse
// LGPL License

#ifndef KDL_HP_CHAINIKSOLVERPOS_NR_JL_HPP
#define KDL_HP_CHAINIKSOLVERPOS_NR_JL_HPP

#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>

namespace KDL {

    class HP_ChainIkSolverPos_NR_JL : public ChainIkSolverPos
    {
    public:
        HP_ChainIkSolverPos_NR_JL(const Chain& chain, const JntArray& q_min, const JntArray& q_max,
                                   ChainFkSolverPos& fksolver, ChainIkSolverVel& iksolver,
                                   unsigned int maxiter = 100, double eps = 1e-6);
        ~HP_ChainIkSolverPos_NR_JL();

        virtual int CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out);
        virtual void updateInternalDataStructures() {}

    private:
        const Chain chain;
        JntArray q_min;
        JntArray q_max;
        ChainFkSolverPos& fksolver;
        ChainIkSolverVel& iksolver;
        JntArray delta_q;
        Frame f;
        Twist delta_twist;

        unsigned int maxiter;
        double eps;
    };
}

#endif

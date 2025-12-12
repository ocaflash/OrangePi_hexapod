// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// LGPL License

#ifndef KDL_SVD_HH_HPP
#define KDL_SVD_HH_HPP

#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <vector>

namespace KDL
{
    class SVD_HH
    {
    public:
        SVD_HH(const Jacobian& jac);
        ~SVD_HH();

        int calculate(const Jacobian& jac, std::vector<JntArray>& U,
                      JntArray& w, std::vector<JntArray>& v, int maxiter);
    private:
        JntArray tmp;
    };
}
#endif

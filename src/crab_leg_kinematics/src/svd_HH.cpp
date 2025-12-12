// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// LGPL License

#include "svd_HH.hpp"
#include <cmath>

namespace KDL
{
    inline double PYTHAG(double a, double b) {
        double at, bt, ct;
        at = std::fabs(a);
        bt = std::fabs(b);
        if (at > bt) {
            ct = bt / at;
            return at * sqrt(1.0 + ct * ct);
        } else {
            if (bt == 0)
                return 0.0;
            else {
                ct = at / bt;
                return bt * sqrt(1.0 + ct * ct);
            }
        }
    }

    inline double SIGN(double a, double b) {
        return ((b) >= 0.0 ? std::fabs(a) : -std::fabs(a));
    }

    SVD_HH::SVD_HH(const Jacobian& jac) : tmp(jac.columns()) {}

    SVD_HH::~SVD_HH() {}

    int SVD_HH::calculate(const Jacobian& jac, std::vector<JntArray>& U, JntArray& w, std::vector<JntArray>& v, int maxiter)
    {
        const int rows = jac.rows();
        const int cols = jac.columns();

        int i(-1), its(-1), j(-1), jj(-1), k(-1), nm = 0;
        int ppi(0);
        bool flag, maxarg1, maxarg2;
        double anorm(0), c(0), f(0), h(0), s(0), scale(0), x(0), y(0), z(0), g(0);

        for (i = 0; i < rows; i++)
            for (j = 0; j < cols; j++)
                U[i](j) = jac(i, j);
        if (rows > cols)
            for (i = rows; i < cols; i++)
                for (j = 0; j < cols; j++)
                    U[i](j) = 0;

        for (i = 0; i < cols; i++) {
            ppi = i + 1;
            tmp(i) = scale * g;
            g = s = scale = 0.0;
            if (i < rows) {
                for (k = i; k < rows; k++) scale += std::fabs(U[k](i));
                if (scale) {
                    for (k = i; k < rows; k++) {
                        U[k](i) /= scale;
                        s += U[k](i) * U[k](i);
                    }
                    f = U[i](i);
                    g = -SIGN(sqrt(s), f);
                    h = f * g - s;
                    U[i](i) = f - g;
                    for (j = ppi; j < cols; j++) {
                        for (s = 0.0, k = i; k < rows; k++) s += U[k](i) * U[k](j);
                        f = s / h;
                        for (k = i; k < rows; k++) U[k](j) += f * U[k](i);
                    }
                    for (k = i; k < rows; k++) U[k](i) *= scale;
                }
            }
            w(i) = scale * g;
            g = s = scale = 0.0;
            if ((i < rows) && (i + 1 != cols)) {
                for (k = ppi; k < cols; k++) scale += std::fabs(U[i](k));
                if (scale) {
                    for (k = ppi; k < cols; k++) {
                        U[i](k) /= scale;
                        s += U[i](k) * U[i](k);
                    }
                    f = U[i](ppi);
                    g = -SIGN(sqrt(s), f);
                    h = f * g - s;
                    U[i](ppi) = f - g;
                    for (k = ppi; k < cols; k++) tmp(k) = U[i](k) / h;
                    for (j = ppi; j < rows; j++) {
                        for (s = 0.0, k = ppi; k < cols; k++) s += U[j](k) * U[i](k);
                        for (k = ppi; k < cols; k++) U[j](k) += s * tmp(k);
                    }
                    for (k = ppi; k < cols; k++) U[i](k) *= scale;
                }
            }
            maxarg1 = anorm;
            maxarg2 = (std::fabs(w(i)) + std::fabs(tmp(i)));
            anorm = maxarg1 > maxarg2 ? maxarg1 : maxarg2;
        }

        for (i = cols - 1; i >= 0; i--) {
            if (i < cols - 1) {
                if (g) {
                    for (j = ppi; j < cols; j++) v[j](i) = (U[i](j) / U[i](ppi)) / g;
                    for (j = ppi; j < cols; j++) {
                        for (s = 0.0, k = ppi; k < cols; k++) s += U[i](k) * v[k](j);
                        for (k = ppi; k < cols; k++) v[k](j) += s * v[k](i);
                    }
                }
                for (j = ppi; j < cols; j++) v[i](j) = v[j](i) = 0.0;
            }
            v[i](i) = 1.0;
            g = tmp(i);
            ppi = i;
        }

        for (i = cols - 1 < rows - 1 ? cols - 1 : rows - 1; i >= 0; i--) {
            ppi = i + 1;
            g = w(i);
            for (j = ppi; j < cols; j++) U[i](j) = 0.0;
            if (g) {
                g = 1.0 / g;
                for (j = ppi; j < cols; j++) {
                    for (s = 0.0, k = ppi; k < rows; k++) s += U[k](i) * U[k](j);
                    f = (s / U[i](i)) * g;
                    for (k = i; k < rows; k++) U[k](j) += f * U[k](i);
                }
                for (j = i; j < rows; j++) U[j](i) *= g;
            } else {
                for (j = i; j < rows; j++) U[j](i) = 0.0;
            }
            ++U[i](i);
        }

        for (k = cols - 1; k >= 0; k--) {
            for (its = 1; its <= maxiter; its++) {
                flag = true;
                for (ppi = k; ppi >= 0; ppi--) {
                    nm = ppi - 1;
                    if ((std::fabs(tmp(ppi)) + anorm) == anorm) {
                        flag = false;
                        break;
                    }
                    if ((std::fabs(w(nm) + anorm) == anorm)) break;
                }
                if (flag) {
                    c = 0.0;
                    s = 1.0;
                    for (i = ppi; i <= k; i++) {
                        f = s * tmp(i);
                        tmp(i) = c * tmp(i);
                        if ((std::fabs(f) + anorm) == anorm) break;
                        g = w(i);
                        h = PYTHAG(f, g);
                        w(i) = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = (-f * h);
                        for (j = 0; j < rows; j++) {
                            y = U[j](nm);
                            z = U[j](i);
                            U[j](nm) = y * c + z * s;
                            U[j](i) = z * c - y * s;
                        }
                    }
                }
                z = w(k);

                if (ppi == k) {
                    if (z < 0.0) {
                        w(k) = -z;
                        for (j = 0; j < cols; j++) v[j](k) = -v[j](k);
                    }
                    break;
                }
                x = w(ppi);
                nm = k - 1;
                y = w(nm);
                g = tmp(nm);
                h = tmp(k);
                f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);

                g = PYTHAG(f, 1.0);
                f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;

                c = s = 1.0;
                for (j = ppi; j <= nm; j++) {
                    i = j + 1;
                    g = tmp(i);
                    y = w(i);
                    h = s * g;
                    g = c * g;
                    z = PYTHAG(f, h);
                    tmp(j) = z;
                    c = f / z;
                    s = h / z;
                    f = x * c + g * s;
                    g = g * c - x * s;
                    h = y * s;
                    y = y * c;
                    for (jj = 0; jj < cols; jj++) {
                        x = v[jj](j);
                        z = v[jj](i);
                        v[jj](j) = x * c + z * s;
                        v[jj](i) = z * c - x * s;
                    }
                    z = PYTHAG(f, h);
                    w(j) = z;
                    if (z) {
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }
                    f = (c * g) + (s * y);
                    x = (c * y) - (s * g);
                    for (jj = 0; jj < rows; jj++) {
                        y = U[jj](j);
                        z = U[jj](i);
                        U[jj](j) = y * c + z * s;
                        U[jj](i) = z * c - y * s;
                    }
                }
                tmp(ppi) = 0.0;
                tmp(k) = f;
                w(k) = x;
            }
        }
        if (its == maxiter)
            return (-2);
        else
            return (0);
    }
}

#include "least_squares_method.h"

namespace virtualgimbal
{

Eigen::VectorXd least_squares_method(Eigen::VectorXd &x, Eigen::VectorXd &y, int degree)
{
    int N = degree + 1;

    assert(x.size() == y.size());
    
    Eigen::MatrixXd A(N, N);

    std::vector<double> sum;
    for (int order = 0,e=2 * degree + 1; order < e; ++order)
    {
        sum.push_back(x.array().pow((double)order).sum());
    }

    for (int col = 0; col < N; ++col)
    {
        for (int row = 0; row < N; ++row)
        {
            A(row, col) = sum[row + col];
        }
    }
    Eigen::VectorXd B(N);
    for (int i = 0; i < N; ++i)
    {
        B(i) = (x.array().pow((double)i) * y.array()).sum();
    }

    return A.inverse() * B;
}

} // namespace virtualgimbal
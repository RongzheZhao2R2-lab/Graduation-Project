#include <Eigen/Eigen>


// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
class BandedSystem
{
public:
// The size of A, as well as the lower/upper
// banded width p/q are needed
inline void create(const int &n, const int &p, const int &q)
{
    // In case of re-creating before destroying
    destroy();
    N = n;
    lowerBw = p;
    upperBw = q;
    int actualSize = N * (lowerBw + upperBw + 1);
    ptrData = new double[actualSize];
    std::fill_n(ptrData, actualSize, 0.0);
    return;
}

inline void destroy()
{
    if (ptrData != nullptr)
    {
        delete[] ptrData;
        ptrData = nullptr;
    }
    return;
}

inline void operator=(const BandedSystem &bs)
{
    ptrData = nullptr;
    create(bs.N, bs.lowerBw, bs.upperBw);
    memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));
}

private:
int N;
int lowerBw;
int upperBw;
double *ptrData = nullptr;

public:
// Reset the matrix to zero
inline void reset(void)
{
    std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
    return;
}

// The band matrix is stored as suggested in "Matrix Computation"
inline const double &operator()(const int &i, const int &j) const
{
    return ptrData[(i - j + upperBw) * N + j];
}

inline double &operator()(const int &i, const int &j)
{
    return ptrData[(i - j + upperBw) * N + j];
}

// This function conducts banded LU factorization in place
// Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
inline void factorizeLU()
{
    int iM, jM;
    double cVl;
    for (int k = 0; k <= N - 2; k++)
    {
        iM = std::min(k + lowerBw, N - 1);
        cVl = operator()(k, k);
        for (int i = k + 1; i <= iM; i++)
        {
            if (operator()(i, k) != 0.0)
            {
                operator()(i, k) /= cVl;
            }
        }
        jM = std::min(k + upperBw, N - 1);
        for (int j = k + 1; j <= jM; j++)
        {
            cVl = operator()(k, j);
            if (cVl != 0.0)
            {
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, j) -= operator()(i, k) * cVl;
                    }
                }
            }
        }
    }
    return;
}

// This function solves Ax=b, then stores x in b
// The input b is required to be N*m, i.e.,
// m vectors to be solved.
inline void solve(const Eigen::MatrixXd &b, Eigen::MatrixXd &res) const
{
    // 将输入 b 复制到 res 中，避免修改原始输入
    res = b;

    int iM;
    // 前向替换 (处理下三角部分 L)
    for (int j = 0; j <= N - 1; j++)
    {
        iM = std::min(j + lowerBw, N - 1);
        for (int i = j + 1; i <= iM; i++)
        {
            if (operator()(i, j) != 0.0)
            {
                res.row(i) -= operator()(i, j) * res.row(j);
            }
        }
    }

    // 后向替换 (处理上三角部分 U)
    for (int j = N - 1; j >= 0; j--)
    {
        res.row(j) /= operator()(j, j); // 除以对角线元素 U[j][j]
        iM = std::max(0, j - upperBw);
        for (int i = iM; i <= j - 1; i++)
        {
            if (operator()(i, j) != 0.0)
            {
                res.row(i) -= operator()(i, j) * res.row(j);
            }
        }
    }
}

// This function solves ATx=b, then stores x in b
// The input b is required to be N*m, i.e.,
// m vectors to be solved.
inline void solveAdj(const Eigen::MatrixXd &b, Eigen::MatrixXd &res) const 
{
    // 复制输入到 res，避免修改原始数据
    res = b;

    int iM;
    // 前向替换（处理转置后的下三角部分 L^T）
    for (int j = 0; j <= N - 1; j++) 
    {
        // 解对角元素
        res.row(j) /= operator()(j, j);
        
        // 更新右方列（对应原矩阵的上三角转置部分）
        iM = std::min(j + upperBw, N - 1);
        for (int i = j + 1; i <= iM; i++) 
        {
            if (operator()(j, i) != 0.0) 
            {
                res.row(i) -= operator()(j, i) * res.row(j);
            }
        }
    }

    // 后向替换（处理转置后的上三角部分 U^T）
    for (int j = N - 1; j >= 0; j--) 
    {
        // 更新左方列（对应原矩阵的下三角转置部分）
        iM = std::max(0, j - lowerBw);
        for (int i = iM; i <= j - 1; i++) 
        {
            if (operator()(j, i) != 0.0) 
            {
                res.row(i) -= operator()(j, i) * res.row(j);
            }
        }
    }
}

EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
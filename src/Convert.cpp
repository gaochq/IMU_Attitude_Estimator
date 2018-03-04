#include "Convert.h"
#include "TypeDefs.h"
#include <fstream>

namespace IMU
{

void Vect_to_SkewMat(Vector_3 Vector, Matrix_3 &Matrix)
{
    Matrix << 0, -Vector(2), Vector(1),
            Vector(2), 0, -Vector(0),
            -Vector(1), Vector(0), 0;
}

void Angular_to_Mat(Vector_3 Vector, Eigen::Matrix<double, 4, 4> &Matrix)
{
    Matrix << 0, -Vector(0), -Vector(1), -Vector(2),
            Vector(0), 0, Vector(2), -Vector(1),
            Vector(1), -Vector(2), 0, Vector(0),
            Vector(2), Vector(1), -Vector(0), 0;
}

Vector_4 Quaternion_to_Vect(Eigen::Quaterniond q)
{
    Vector_4 vector;
    vector(0) = q.w();
    vector.block<3, 1>(1, 0) = q.vec();

    return vector;
}

Eigen::Quaterniond QuatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
    Eigen::Quaterniond resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}

Matrix_3 Quat_to_Matrix(Eigen::Quaterniond q)
{
    Matrix_3 matrix;
    matrix << q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(), 2 * (q.x()*q.y() - q.w()*q.z()), 2 * (q.x()*q.z() + q.w()*q.y()),
            2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(), 2 * (q.y()*q.z() - q.w()*q.x()),
            2 * (q.x()*q.z() - q.w()*q.y()), 2 * (q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();

    return matrix;
}

Eigen::MatrixXd readFromfile(const string file_name)
{
    Eigen::MatrixXd matrix;
    std::vector<double> entries;
    ifstream data(file_name, ios::binary);
    string lineOfData;

    if (data.is_open())
    {
        int i = 0;
        int cols = 0;
        while (data.good())
        {
            int j = 0;
            getline(data, lineOfData);
            stringstream stream(lineOfData);
            while (!stream.eof())
            {
                double a;
                stream >> a;
                entries.push_back(a);
                j++;
            }
            cols = j;
            i++;
        }
        matrix = Eigen::MatrixXd::Map(&entries[0], cols, i).transpose();

        return matrix;
    }
    else
    {
        cout << "Unable to open file" << std::endl;

        return Eigen::Vector3d::Zero();
    }

}

bool writeTofile(Eigen::MatrixXd matrix, const string file_name)
{
    std::ofstream file(file_name, ios::binary);
    if (file.is_open())
    {
        file << matrix << '\n';
    }
    else
        return false;
    file.close();
    return true;
}

Matrix_3 Euler_to_RoatMat(Vector_3 Euler)
{
    Matrix_3 Rotate_Mat;
    double theta;
    Matrix_3 Skew_Euler;

    theta = Euler.norm();
    Euler.normalize();

    Vect_to_SkewMat(Euler, Skew_Euler);

    Rotate_Mat = Matrix_3::Identity() + sin(theta)*Skew_Euler + (1 - cos(theta))*Skew_Euler.transpose()*Skew_Euler;
    return Rotate_Mat;
}

Eigen::Quaterniond Euler_to_Quaternion(Vector_3 Euler)
{
    Eigen::Quaterniond quaternion;

    /*
    double theta = sqrt(Euler(0)*Euler(0) + Euler(1)*Euler(1) + Euler(2)*Euler(2));
    Euler.normalize();

    quaternion.w() = cos(0.5*theta);
    quaternion.vec() = Euler*sin(0.5*theta);
    */



    // using Eigen
    /*
    quaternion = Eigen::AngleAxisd(Euler(0), Eigen::Vector3d::UnitX())

    * Eigen::AngleAxisd(Euler(1), Eigen::Vector3d::UnitY())

    *Eigen::AngleAxisd(Euler(2), Eigen::Vector3d::UnitZ());
    */
    double CosRoll = cos(Euler[0] * 0.5);
    double SinRoll = sin(Euler[0] * 0.5);
    double CosPitch = cos(Euler[1] * 0.5);
    double SinPitch = sin(Euler[1] * 0.5);
    double CosYaw = cos(Euler[2] * 0.5);
    double SinYaw = sin(Euler[2] * 0.5);

    double w = CosRoll*CosPitch*CosYaw + SinRoll*SinPitch*SinYaw;
    double x = CosPitch*SinRoll*CosYaw - CosRoll*SinPitch*SinYaw;
    double y = CosRoll*CosYaw*SinPitch + SinRoll*CosPitch*SinYaw;
    double z = CosRoll*CosPitch*SinYaw - CosYaw*SinPitch*SinRoll;

    quaternion = Eigen::Quaterniond(w, x, y, z);

    return quaternion;
}

Vector_3 Quaternion_to_Euler(Eigen::Quaterniond q)
{
    Vector_3 Euler;

    // the normal way

    Euler(0) = atan2(2 * (q.y()*q.z() + q.w()*q.x()), (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z()));
    Euler(1) = asin(-2 * q.x()*q.z() + 2 * q.w()*q.y());
    Euler(2) = atan2(2 * (q.x()*q.y() + q.w()*q.z()), (q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z())) - 8.3*M_PI/180;

    /*
    // using the eigen

    q.normalize();

    Euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    */
    return Euler*180/M_PI;
}

Vector_3 Rotation_to_Euler(Matrix_3 Rotation)
{
    Vector_3 Euler;

    Euler(0) = atan2(Rotation(1, 2), Rotation(2, 2));
    Euler(1) = -asin(Rotation(0, 2));
    Euler(2) = atan2(Rotation(0, 1), Rotation(0, 0));

    return Euler;
}

Eigen::Quaterniond Rotation_to_Quater(Matrix_3 Rotation)
{
    Eigen::Quaterniond quaternion;
    Vector_4 quat_tmp;
    double tr;
    tr = Rotation.trace();
    if (tr > 0.0)
    {
        double s = sqrtf(tr + 1.0);
        quat_tmp(0) = s*0.5;
        s = 0.5 / s;
        quat_tmp(1) = (Rotation(2, 1) - Rotation(1, 2))*s;
        quat_tmp(2) = (Rotation(0, 2) - Rotation(2, 0))*s;
        quat_tmp(3) = (Rotation(1, 0) - Rotation(0, 1))*s;
    }
    else
    {
        int dcm_i = 0;
        for (int i = 1; i < 3; i++)
        {
            if (Rotation(i, i) > Rotation(dcm_i, dcm_i))
                dcm_i = i;
        }
        int dcm_j = (dcm_i + 1) % 3;
        int dcm_k = (dcm_i + 2) % 3;
        double s = sqrtf((Rotation(dcm_i, dcm_i) - Rotation(dcm_j, dcm_j) -
                          Rotation(dcm_k, dcm_k)) + 1.0f);

        quat_tmp(dcm_i + 1) = s*0.5;
        s = 0.5 / s;
        quat_tmp(dcm_j + 1) = (Rotation(dcm_i, dcm_j) + Rotation(dcm_j, dcm_i)) * s;
        quat_tmp[dcm_k + 1] = (Rotation(dcm_k, dcm_i) + Rotation(dcm_i, dcm_k)) * s;
        quat_tmp[0] = (Rotation(dcm_k, dcm_j) - Rotation(dcm_j, dcm_k)) * s;
    }
    quaternion.w() = quat_tmp(0);
    quaternion.vec() = quat_tmp.block<3, 1>(1, 0);

    return quaternion;
}

Eigen::Quaterniond BuildUpdateQuat(Eigen::Vector3d DeltaTheta)
{
    Eigen::Vector3d DeltaQuat = 0.5*DeltaTheta;
    double checknorm = DeltaQuat.transpose()*DeltaQuat;

    Eigen::Quaterniond UpdateQuat;
    if (checknorm > 1)
    {
        UpdateQuat = Eigen::Quaterniond(1, DeltaQuat[0], DeltaQuat[1], DeltaQuat[2]);
        UpdateQuat = UpdateQuat.coeffs()*sqrt(1 + checknorm);
    }
    else
        UpdateQuat = Eigen::Quaterniond(sqrt(1 - checknorm), DeltaQuat[0], DeltaQuat[1], DeltaQuat[2]);

    UpdateQuat.normalize();

    return  UpdateQuat;
}

} //IMU
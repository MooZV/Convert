#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

bool convert(const std::string &convert_option, std::string &convert_file_name);
bool rotationvector2X(std::ifstream &convert_file);
bool rotationMat2X(std::ifstream &convert_file);
bool eulerAngle2X(std::ifstream &convert_file);
bool quaternion2X(std::ifstream &convert_file);
void split(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters);
Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);


int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Usage: convert.sh options convert_file_dir";
        return -1;
    }

    std::string convert_option(argv[1]);
    std::cout << "convert option: " << convert_option;
    std::string convert_file_name(argv[2]);
    // std::cout << "file dir: " << file_dir;
    // std::string convert_file_name(file_dir + "/convert.txt");

    if (!convert(convert_option, convert_file_name)) {
        std::cout << "Convert failed";
        return -1;
    } 
    
    return 0;
}


bool convert(const std::string &convert_option, std::string &convert_file_name) {
    std::ifstream convert_file(convert_file_name);

    if (!convert_file.is_open()) {
        std::cout << "Failed to open convert file! File name: " << convert_file_name;
        return false;
    }
    if (convert_option == "h") {
        std::cout << "Usage: convert.sh options convert_file_dir";
        std::cout << "------------------------";
        std::cout << "-h | help info";
        std::cout << "-v | rotationvector2X";
        std::cout << "-m | rotationMat2X";
        std::cout << "-e | eulerAngle2X";
        std::cout << "-q | quaternion2X" ; 
    }
    else if (convert_option == "-v") {
        rotationvector2X(convert_file);
    }
    else if (convert_option == "-m") {
        rotationMat2X(convert_file);
    }
    else if (convert_option == "-e") {
        eulerAngle2X(convert_file);
    }
    else if (convert_option == "-q") {
        quaternion2X(convert_file);
    }
    else {
        std::cout << "pls set right flagCreateBoard;";
    }

    return true;
}

bool rotationvector2X(std::ifstream &convert_file) {
    std::map<std::string, double> vect;
    std::string str;
    int count = 0;
    bool find_first = false;
    while (getline(convert_file, str)) {
        if (str.find("rotationvect") != std::string::npos) {
            find_first = true;
            count = 0;
            continue;
        }
        std::vector<std::string> str_list;
        if (!find_first) {
            continue;
        }
        else if (count < 4) {
            split(str, str_list, ":");
            if (str_list.size() != 2) {
                std::cout << "Error in extract valid info from convert.txt ";
                return false;
            }
            str_list[0].erase(0, str_list[0].find_first_not_of(" "));
            str_list[0].erase(str_list[0].find_last_not_of(" ") + 1);
            str_list[1].erase(0, str_list[1].find_first_not_of(" "));
            str_list[1].erase(str_list[1].find_last_not_of(" ") + 1);
            vect[str_list[0]] = stod(str_list[1]);
            count++;
        }
        else {
            break;
        }
    }
    convert_file.close();

    // init rotationVector
    Eigen::AngleAxisd rotationVector(vect["alpha"], Eigen::Vector3d(vect["x"], vect["y"], vect["z"]));
    // Vector3d(x,y,z)
    std::cout << "rotationVector.axis: \n" << rotationVector.axis().transpose() << std::endl;
    std::cout << "rotationVector.angle: \n" << rotationVector.angle() << std::endl;

    // rotationVector --> rotationMatrix
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = rotationVector.matrix();
    std::cout << "rotationMatrix: \n" << rotationMatrix << std::endl;

    // rotationVector --> quaternion
    Eigen::Quaterniond quaternion;
    quaternion = rotationVector;
    std::cout << "Quaterniond--[x,y,z,w]: \n"<< quaternion.coeffs() << std::endl;

    // rotationVector --> eulerAngle(Z-Y-X:  YPR))
    Eigen::Vector3d eulerAngle;
    eulerAngle = toEulerAngle(quaternion, eulerAngle(2), eulerAngle(1), eulerAngle(0));
    // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
    std::cout << "eulerAngle--[yaw,pitch,roll]: \n" << eulerAngle.transpose() << std::endl;

    return true;
}


bool rotationMat2X(std::ifstream &convert_file) {
    std::map<std::string, double> matrix;
    std::string str;
    int count = 0;
    bool find_first = false;
    while (getline(convert_file, str)) {
        if (str.find("rotationMat") != std::string::npos) {
            find_first = true;
            count = 0;
            continue;
        }
        std::vector<std::string> str_list;
        if (!find_first) {
            continue;
        }
        else if (count < 9) {
            split(str, str_list, ":");
            if (str_list.size() != 2) {
                std::cout << "Error in extract valid info from convert.txt ";
                return false;
            }
            str_list[0].erase(0, str_list[0].find_first_not_of(" "));
            str_list[0].erase(str_list[0].find_last_not_of(" ") + 1);
            str_list[1].erase(0, str_list[1].find_first_not_of(" "));
            str_list[1].erase(str_list[1].find_last_not_of(" ") + 1);
            matrix[str_list[0]] = stod(str_list[1]);
            count++;
        }
        else {
            break;
        }
    }
    convert_file.close();

    // init rotationMatrix
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << matrix["x_00"], matrix["x_01"], matrix["x_02"],
                      matrix["x_10"], matrix["x_11"], matrix["x_12"],
                      matrix["x_20"], matrix["x_21"], matrix["x_22"];
    // rotationMatrix << x_00,x_01,x_02,x_10,x_11,x_12,x_20,x_21,x_22;
    std::cout << "rotationMatrix: \n" << rotationMatrix << std::endl;

    // rotationMatrix --> rotationVector
    Eigen::AngleAxisd rotationVector;
    rotationVector = rotationMatrix;
    std::cout << "rotationVector.axis: \n" << rotationVector.axis().transpose() << std::endl;
    std::cout << "rotationVector.angle: \n" << rotationVector.angle() << std::endl;

    // rotationMatrix --> quaternion
    Eigen::Quaterniond quaternion;
    quaternion = rotationMatrix;
    std::cout << "Quaterniond--[x,y,z,w]: \n"<< quaternion.coeffs() << std::endl;

    // rotationMatrix --> eulerAngle(Z-Y-X:  YPR))
    Eigen::Vector3d eulerAngle;
    eulerAngle = toEulerAngle(quaternion, eulerAngle(2), eulerAngle(1), eulerAngle(0));
    // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
    std::cout << "eulerAngle--[yaw,pitch,roll]: \n" << eulerAngle.transpose() << std::endl;

    return true;
}

bool eulerAngle2X(std::ifstream &convert_file) {

    std::map<std::string, double> euler;
    std::string str;
    int count = 0;
    bool find_first = false;

    while (getline(convert_file, str)) {
        if (str.find("eulerAngle") != std::string::npos) {
            find_first = true;
            count = 0;
            continue;
        }
        std::vector<std::string> str_list;
        if (!find_first) {
            continue;
        }
        else if (count < 3) {
            split(str, str_list, ":");
            if (str_list.size() != 2) {
                std::cout << "Error in extract valid info from convert.txt ";
                return false;
            }
            str_list[0].erase(0, str_list[0].find_first_not_of(" "));
            str_list[0].erase(str_list[0].find_last_not_of(" ") + 1);
            str_list[1].erase(0, str_list[1].find_first_not_of(" "));
            str_list[1].erase(str_list[1].find_last_not_of(" ") + 1);
            euler[str_list[0]] = stod(str_list[1]);
            count++;
        }
        else {
            break;
        }
    }
    convert_file.close();

    // init eulerAngle
    // Eigen::Vector3d eulerAngle(1.55091858, -3.11521959, 1.62878323);
    Eigen::Vector3d eulerAngle(euler["yaw"], euler["pitch"],euler["roll"]);
    // Eigen::Vector3d eulerAngle(yaw,pitch,roll);
    std::cout << "eulerAngle--[yaw,pitch,roll]: \n" << eulerAngle.transpose() << std::endl;

    // eulerAngle --> rotationVector
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ())); 
    Eigen::AngleAxisd rotationVector;
    rotationVector = yawAngle * pitchAngle * rollAngle;
    std::cout << "rotationVector.axis: \n" << rotationVector.axis().transpose() << std::endl;
    std::cout << "rotationVector.angle: \n" << rotationVector.angle() << std::endl;

    // eulerAngle --> rotationMatrix; 
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = yawAngle * pitchAngle * rollAngle;
    std::cout << "rotationMatrix: \n" << rotationMatrix << std::endl;

    // eulerAngle --> quaternion
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;
    std::cout << "Quaterniond--[x,y,z,w]: \n" << quaternion.coeffs() << std::endl;
    
    return true;
}

bool quaternion2X(std::ifstream &convert_file) {

    std::map<std::string, double> quat;
    std::string str;
    int count = 0;
    bool find_first = false;

    while (getline(convert_file, str)) {
        if (str.find("quaternion") != std::string::npos) {
            find_first = true;
            count = 0;
            continue;
        }
        std::vector<std::string> str_list;
        if (!find_first) {
            continue;
        }
        else if (count < 4) {
            split(str, str_list, ":");
            if (str_list.size() != 2) {
                std::cout << "Error in extract valid info from convert.txt ";
                return false;
            }
            str_list[0].erase(0, str_list[0].find_first_not_of(" "));
            str_list[0].erase(str_list[0].find_last_not_of(" ") + 1);
            str_list[1].erase(0, str_list[1].find_first_not_of(" "));
            str_list[1].erase(str_list[1].find_last_not_of(" ") + 1);
            quat[str_list[0]] = stod(str_list[1]);
            count++;
        }
        else {
            break;
        }
    }
    convert_file.close();

    // init quaternion
    Eigen::Quaterniond quaternion(quat["w"], quat["x"], quat["y"], quat["z"]);
    // Eigen::Quaterniond quaternion(w,x,y,z);
    std::cout << "Quaterniond--[x,y,z,w]: \n" << quaternion.coeffs() << std::endl;

    // quaternion --> rotationVector
    Eigen::AngleAxisd rotationVector;
    rotationVector = quaternion;
    std::cout << "rotationVector.axis: \n" << rotationVector.axis().transpose() << std::endl;
    std::cout << "rotationVector.angle: \n" << rotationVector.angle() << std::endl;

    // quaternion --> rotationMatrix
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = quaternion.matrix();
    std::cout << "rotationMatrix: \n" << rotationMatrix << std::endl;

    // quaternion --> eulerAngle
    Eigen::Vector3d eulerAngle;
    eulerAngle = toEulerAngle(quaternion, eulerAngle(2), eulerAngle(1), eulerAngle(0));
    // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
    std::cout << "eulerAngle--[yaw,pitch,roll]: \n" << eulerAngle.transpose() << std::endl;

    return true;
}

Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
    Eigen::Vector3d eulerAngle;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);
    eulerAngle(2) = roll;

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
    pitch = asin(sinp);
    eulerAngle(1) = pitch;

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
    eulerAngle(0) = yaw;
    return eulerAngle;
}


void split(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters) {
    tokens.clear();
    std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    std::string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos) {
        tokens.emplace_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}

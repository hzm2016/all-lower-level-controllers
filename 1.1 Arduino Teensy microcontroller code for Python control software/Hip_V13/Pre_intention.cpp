#include <iostream>
#include <vector>
#include <cmath>
// #include <Eigen/Dense>  

#include <ArduinoEigenDense.h>

using namespace std;
using namespace Eigen;

// 定义一个简单的结构体来存储高斯分量参数
struct GaussianComponent {
    VectorXd mean; // 均值向量
    MatrixXd covariance; // 协方差矩阵
    double weight; // 权重
};

// GMM 类
class OnlineGMM {
private:
    vector<GaussianComponent> components; // 高斯混合模型的分量
    double alpha; // 学习率（更新新数据的权重）

public:
    OnlineGMM(int num_components, int dim, double alpha = 0.1) : alpha(alpha) {
        // 初始化GMM分量
        components.resize(num_components);
        for (auto &component : components) {
            component.mean = VectorXd::Zero(dim);
            component.covariance = MatrixXd::Identity(dim, dim);
            component.weight = 1.0 / num_components; 
        }
    }

    // 更新GMM参数
    void update(const VectorXd &x) {
        // 计算每个分量的响应度（责任值）
        vector<double> responsibilities(components.size());
        double total_responsibility = 0.0;

        for (size_t i = 0; i < components.size(); i++) {
            responsibilities[i] = components[i].weight * gaussianPDF(x, components[i]);
            total_responsibility += responsibilities[i];
        }

        // 归一化责任值
        for (auto &r : responsibilities) {
            r /= total_responsibility;
        }

        // 更新每个分量
        for (size_t i = 0; i < components.size(); i++) {
            double r = responsibilities[i];
            GaussianComponent &comp = components[i];

            // 更新权重
            comp.weight = (1 - alpha) * comp.weight + alpha * r;

            // 更新均值
            VectorXd diff = x - comp.mean;
            comp.mean += alpha * r * diff;

            // 更新协方差
            comp.covariance = (1 - alpha) * comp.covariance + alpha * r * (diff * diff.transpose());
        }
    }

    // 使用GMR进行回归
    VectorXd predict(const VectorXd &input, const VectorXi &input_dims, const VectorXi &output_dims) {
        VectorXd predicted_output = VectorXd::Zero(output_dims.size());
        double total_weight = 0.0;

        for (const auto &comp : components) {
            // 提取输入和输出相关的部分
            auto input_mean = extractDims(comp.mean, input_dims);
            auto output_mean = extractDims(comp.mean, output_dims);

            auto input_cov = extractDims(comp.covariance, input_dims, input_dims);
            auto cross_cov = extractDims(comp.covariance, input_dims, output_dims);

            // 计算回归
            VectorXd gain = cross_cov.transpose() * input_cov.inverse();
            VectorXd local_prediction = output_mean + gain * (input - input_mean);

            double likelihood = gaussianPDF(input, comp); 
            // predicted_output += likelihood * local_prediction;
            // total_weight += likelihood;
        }

        return predicted_output / total_weight;
    }

private:
    // 高斯PDF计算
    double gaussianPDF(const VectorXd &x, const GaussianComponent &comp) {
        VectorXd diff = x - comp.mean;
        double exponent = -0.5 * diff.transpose() * comp.covariance.inverse() * diff;
        double norm = std::pow(2 * M_PI, -x.size() / 2.0) * std::sqrt(comp.covariance.determinant());
        return std::exp(exponent) / norm;
    }

    // 辅助函数：提取向量的指定维度
    VectorXd extractDims(const VectorXd &vec, const VectorXi &dims) {
        VectorXd result;
        result.setZero(dims.size());
        for (int i = 0; i < dims.size(); i++) {
            result(i) = vec(dims(i));
        }
        return result; 
    }

    // 辅助函数：提取矩阵的指定维度
    MatrixXd extractDims(const MatrixXd &mat, const VectorXi &rows, const VectorXi &cols) {
        MatrixXd result;  
        result.setZero(rows.size(), cols.size());   
        for (int i = 0; i < rows.size(); i++) {
            for (int j = 0; j < cols.size(); j++) {
                result(i, j) = mat(rows(i), cols(j));
            }
        }
        return result;
    }
};


int main() {
    // 示例用法
    int dim = 2; // 数据维度
    int num_components = 3; // 高斯分量数

    cout << "I am Here :" << endl; 
    OnlineGMM gmm(num_components, dim);

    // 模拟数据点更新
    VectorXd point(2);
    point << 1.0, 2.0;
    gmm.update(point);

    point << 2.0, 3.0;
    gmm.update(point);

    // 回归预测
    VectorXi input_dims(1), output_dims(1);
    input_dims << 0;
    output_dims << 1;

    VectorXd input(1);
    input << 1.5;

    // auto input_mean = gmm.extractDims(gmm.components[0].mean, input_dims);   
    
    VectorXd predicted_output = gmm.predict(input, input_dims, output_dims); 
    // cout << "Predicted output: " << predicted_output.transpose() << endl;     
    cout << "Predicted output: " << endl;  

    return 0;
}   
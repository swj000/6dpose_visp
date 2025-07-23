#include "sogang.h"
#include "TcpClient.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpMath.h>

QR_CORNER_FORMAT parseQRData(const std::string& text, const std::string& targetQRID) {
    std::stringstream ss(text);
    std::string entry;

    while (std::getline(ss, entry, ',')) {
        std::stringstream entry_stream(entry);
        std::string qr_id;
        std::getline(entry_stream, qr_id, ':');

        if (qr_id != targetQRID)
            continue;

        QR_CORNER_FORMAT qr;
        qr.id = std::stoul(qr_id);

        for (int i = 0; i < 4; ++i) {
            std::string coord_pair;
            if (!std::getline(entry_stream, coord_pair, ':'))
                throw std::runtime_error("좌표 파싱 오류: ID=" + qr_id);

            size_t sep = coord_pair.find('/');
            if (sep == std::string::npos)
                throw std::runtime_error("좌표 형식 오류: " + coord_pair);

            qr.x[i] = std::stoi(coord_pair.substr(0, sep));
            qr.y[i] = std::stoi(coord_pair.substr(sep + 1));
        }

        return qr;
    }

    throw std::runtime_error("QR ID '" + targetQRID + "'가 데이터에 존재하지 않습니다.");
}

std::string getQRDataFromServer(const std::string& ip, int port, const std::string& command) {
    TcpClient client(ip, port);

    if (!client.connectToServer())
        throw std::runtime_error("서버 연결 실패");

    if (!client.sendCommand(command))
        throw std::runtime_error("명령어 전송 실패");

    std::string response = client.receiveResponse();
    if (response.empty())
        throw std::runtime_error("서버로부터 응답 없음");

    return response;
}

std::string getQRDataFromTextFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filePath);
    }

    std::string content((std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());

    file.close();
    return content;
}

void saveHomogeneousMatrix(const vpHomogeneousMatrix& mat, const std::string& filepath) {
    std::ofstream file(filepath);
    if (!file.is_open())
        throw std::runtime_error("보정행렬 파일 저장 실패: " + filepath);

    for (unsigned int i = 0; i < 4; ++i) {
        for (unsigned int j = 0; j < 4; ++j) {
            file << mat[i][j];
            if (j < 3) file << " ";
        }
        file << "\n";
    }
    file.close();
}

vpHomogeneousMatrix loadHomogeneousMatrix(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) throw std::runtime_error("보정행렬 파일을 열 수 없습니다.");

    vpHomogeneousMatrix mat;
    for (unsigned int i = 0; i < 4; ++i)
        for (unsigned int j = 0; j < 4; ++j)
            file >> mat[i][j];

    file.close();
    return mat;
}

vpHomogeneousMatrix computeQRPose(const QR_CORNER_FORMAT& qr, const vpCameraParameters& cam, float qr_size) {
    std::vector<vpPoint> object_points = {
        vpPoint(0, 0, 0),
        vpPoint(qr_size, 0, 0),
        vpPoint(qr_size, qr_size, 0),
        vpPoint(0, qr_size, 0)
    };

    vpPose pose;
    for (size_t i = 0; i < 4; ++i) {
        double x = 0, y = 0;
        vpImagePoint img_pt(qr.y[i], qr.x[i]);
        vpPixelMeterConversion::convertPoint(cam, img_pt, x, y);
        object_points[i].set_x(x);
        object_points[i].set_y(y);
        pose.addPoint(object_points[i]);
    }

    vpHomogeneousMatrix cMo;
    pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);
    return cMo;
}

vpHomogeneousMatrix computeCorrectionMatrix(const vpHomogeneousMatrix& T_qrr1, const vpHomogeneousMatrix& T_qrr2) {
    return T_qrr2 * T_qrr1.inverse();
}

vpHomogeneousMatrix compute6DTP(const QR_CORNER_FORMAT& qr_map, const vpTranslationVector& translation_mm, const vpRxyzVector& rotation_deg) {
    vpCameraParameters cam_qrr(6187.0, 6187.0, 1024.0, 768.0);
    float qr_size = 30.0f;
    vpHomogeneousMatrix T_qrr_qr = computeQRPose(qr_map, cam_qrr, qr_size);

    vpHomogeneousMatrix T_base_qr;
    T_base_qr.buildFrom(
        translation_mm[0], translation_mm[1], translation_mm[2],
        vpMath::rad(rotation_deg[0]),
        vpMath::rad(rotation_deg[1]),
        vpMath::rad(rotation_deg[2])
    );

    return T_qrr_qr * T_base_qr.inverse();
}

vpHomogeneousMatrix apply6DTP(const QR_CORNER_FORMAT& qr_map, const vpHomogeneousMatrix& T_qrr_base) {
    vpCameraParameters cam_qrr(6187.0, 6187.0, 1024.0, 768.0);
    float qr_size = 30.0f;
    vpHomogeneousMatrix T_qrr_qr2 = computeQRPose(qr_map, cam_qrr, qr_size);
    return T_qrr_base.inverse() * T_qrr_qr2;
}

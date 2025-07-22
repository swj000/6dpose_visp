#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <iomanip>

#include "TcpClient.h"

// --- UTILS ---

std::map<std::string, std::vector<vpImagePoint>> parseQRData(const std::string& text) {
    std::map<std::string, std::vector<vpImagePoint>> qr_data;
    std::stringstream ss(text);
    std::string entry;

    while (std::getline(ss, entry, ',')) {
        std::stringstream entry_stream(entry);
        std::string qr_id;
        std::getline(entry_stream, qr_id, ':');

        std::vector<vpImagePoint> points;
        for (int i = 0; i < 4; ++i) {
            std::string coord_pair;
            std::getline(entry_stream, coord_pair, ':');
            size_t sep = coord_pair.find('/');
            int x = std::stoi(coord_pair.substr(0, sep));
            int y = std::stoi(coord_pair.substr(sep + 1));
            points.emplace_back(y, x);  // (v, u) 순서
        }
        qr_data[qr_id] = points;
    }

    return qr_data;
}

std::string getQRDataFromServer(const std::string& ip, int port, const std::string& command = "LON\r\n") {
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

void printPose6D(const std::string& title, const vpHomogeneousMatrix& T) {
    vpTranslationVector t = T.getTranslationVector();
    vpRotationMatrix R = T.getRotationMatrix();

    vpThetaUVector theta_u;
    theta_u.buildFrom(R);

    double angle_deg = vpMath::deg(theta_u.getTheta());
    vpColVector axis = theta_u.getU();

    std::cout << "\n[" << title << "]\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Tx: " << t[0] << " mm\n";
    std::cout << "Ty: " << t[1] << " mm\n";
    std::cout << "Tz: " << t[2] << " mm\n";
    std::cout << "Rot angle: " << angle_deg << " deg\n";
    std::cout << "Rot axis : (" << axis[0] << ", " << axis[1] << ", " << axis[2] << ")\n";
}

// --- pose 추정 ---
vpHomogeneousMatrix computePose(const std::vector<vpImagePoint>& image_points,
    const std::vector<vpPoint>& object_points,
    const vpCameraParameters& cam) {
    vpPose pose;
    std::vector<vpPoint> pts = object_points;

    for (size_t i = 0; i < pts.size(); ++i) {
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(cam, image_points[i], x, y);
        pts[i].set_x(x);
        pts[i].set_y(y);
        pose.addPoint(pts[i]);
    }

    vpHomogeneousMatrix cMo;
    pose.computePose(vpPose::DEMENTHON, cMo);
    // pose.computePose(vpPose::VIRTUAL_VS, cMo);
    return cMo;
}

// opencv 사용할시
//vpHomogeneousMatrix computePose(const std::vector<vpImagePoint>& image_points,
//    const std::vector<vpPoint>& object_points,
//    const vpCameraParameters& cam) {
//    assert(image_points.size() == object_points.size());
//    assert(image_points.size() >= 4);  // solvePnP requires at least 4 points
//
//    // 1. Convert to OpenCV types
//    std::vector<cv::Point2f> cv_image_points;
//    std::vector<cv::Point3f> cv_object_points;
//
//    for (size_t i = 0; i < image_points.size(); ++i) {
//        cv_image_points.emplace_back(image_points[i].get_u(), image_points[i].get_v());
//        cv_object_points.emplace_back(object_points[i].get_oX(),
//            object_points[i].get_oY(),
//            object_points[i].get_oZ());
//    }
//
//    // 2. Convert camera intrinsics to OpenCV
//    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
//        cam.get_px(), 0, cam.get_u0(),
//        0, cam.get_py(), cam.get_v0(),
//        0, 0, 1);
//
//    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);  // assume no distortion
//
//    // 3. Pose estimation
//    cv::Mat rvec, tvec;
//    cv::solvePnP(cv_object_points, cv_image_points, cameraMatrix, distCoeffs,
//        rvec, tvec, false, cv::SOLVEPNP_IPPE);
//
//    // 4. Convert to vpHomogeneousMatrix
//    cv::Mat R;
//    cv::Rodrigues(rvec, R);
//
//    vpRotationMatrix vR;
//    for (int i = 0; i < 3; ++i)
//        for (int j = 0; j < 3; ++j)
//            vR[i][j] = R.at<double>(i, j);
//
//    vpTranslationVector vT(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
//
//    vpHomogeneousMatrix cMo;
//    cMo.buildFrom(vT, vR);
//    return cMo;
//}

vpHomogeneousMatrix computeCorrectionMatrix(const std::map<std::string, std::vector<vpImagePoint>>& qrr1_map, const std::map<std::string, std::vector<vpImagePoint>>& qrr2_map) {
    std::string target_id = "001";

    float qr_size = 30.0f;
    std::vector<vpPoint> object_points = {
        vpPoint(0, 0, 0),
        vpPoint(qr_size, 0, 0),
        vpPoint(qr_size, qr_size, 0),
        vpPoint(0, qr_size, 0)
    };

    // --- QRR1 카메라 내부 파라미터 ---
    vpCameraParameters cam_qrr1(6187.0, 6187.0, 1024.0, 768.0);

    // --- QRR2 카메라 내부 파라미터 ---
    vpCameraParameters cam_qrr2(6670.0, 6670.0, 1024.0, 768.0);

    if (qrr1_map.find(target_id) == qrr1_map.end())
        throw std::runtime_error("QRR1: QR ID " + target_id + " not found.");
    if (qrr2_map.find(target_id) == qrr2_map.end())
        throw std::runtime_error("QRR2: QR ID " + target_id + " not found.");

    // --- Pose 추정 ---
    vpHomogeneousMatrix T_qrr1 = computePose(qrr1_map.at(target_id), object_points, cam_qrr1);
    vpHomogeneousMatrix T_qrr2 = computePose(qrr2_map.at(target_id), object_points, cam_qrr2);

    // --- 보정행렬 계산 ---
    vpHomogeneousMatrix correction_matrix = T_qrr2 * T_qrr1.inverse();
    return correction_matrix;
}

vpHomogeneousMatrix compute6DTP(const vpHomogeneousMatrix& T_base_qr,
    const vpHomogeneousMatrix& T_qrr_qr) {
    // QRR 기준 BASE의 pose = QRR→QR * QR→BASE
    vpHomogeneousMatrix T_qrr_base = T_qrr_qr * T_base_qr.inverse();
    return T_qrr_base;
}

// --- 1807 보정 함수 ---
void runReprojectionPipeline(const std::map<std::string, std::vector<vpImagePoint>>& qrr1_map, const std::map<std::string, std::vector<vpImagePoint>>& qrr2_map, const vpHomogeneousMatrix& correction) {

    std::string qr_id = "1807";

    vpCameraParameters cam_qrr1(6187.0, 6187.0, 1024.0, 768.0);

    // target camera parameter (QRR2)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 6670.0, 0, 1024.0,
        0, 6670.0, 768.0,
        0, 0, 1);
    cv::Mat distCoeffs;

    float qr_size = 28.0f;
    std::vector<vpPoint> object_points = {
        vpPoint(0, 0, 0),
        vpPoint(qr_size, 0, 0),
        vpPoint(qr_size, qr_size, 0),
        vpPoint(0, qr_size, 0)
    };

    if (qrr1_map.find(qr_id) == qrr1_map.end())
        throw std::runtime_error("QRR1: QR ID " + qr_id + " not found.");
    if (qrr2_map.find(qr_id) == qrr2_map.end())
        throw std::runtime_error("QRR2: QR ID " + qr_id + " not found.");

    // QRR1의 pose → QRR2의 pose
    vpHomogeneousMatrix pose_qrr1 = computePose(qrr1_map.at(qr_id), object_points, cam_qrr1);
    vpHomogeneousMatrix pose_qrr2 = correction * pose_qrr1;

    // OpenCV 형식으로 변환
    cv::Mat R_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_cv.at<double>(i, j) = pose_qrr2[i][j];

    cv::Mat rvec, tvec;
    cv::Rodrigues(R_cv, rvec);
    tvec = (cv::Mat_<double>(3, 1) << pose_qrr2[0][3], pose_qrr2[1][3], pose_qrr2[2][3]);

    // --- 6D Pose 출력 ---
    std::cout << "\n[Pose in QRR2 Frame (Translation in mm, Rotation in deg)]\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Tx: " << tvec.at<double>(0) << " mm\n"
        << "Ty: " << tvec.at<double>(1) << " mm\n"
        << "Tz: " << tvec.at<double>(2) << " mm\n";

    cv::Mat rvec_deg = rvec * (180.0 / CV_PI);
    std::cout << "Rx: " << rvec_deg.at<double>(0) << " deg\n"
        << "Ry: " << rvec_deg.at<double>(1) << " deg\n"
        << "Rz: " << rvec_deg.at<double>(2) << " deg\n";

    // --- Reprojection ---
    std::vector<cv::Point3f> object_cv;
    for (const auto& p : object_points)
        object_cv.emplace_back(p.get_oX(), p.get_oY(), p.get_oZ());

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_cv, rvec, tvec, K, distCoeffs, projected_points);

    std::vector<cv::Point2f> ground_truth;
    for (const auto& pt : qrr2_map.at(qr_id))
        ground_truth.emplace_back(pt.get_u(), pt.get_v());

    std::cout << "\n[Reprojection Result - Using QRR2 Ground Truth]\n";
    double total_error = 0.0;
    for (size_t i = 0; i < projected_points.size(); ++i) {
        double err = cv::norm(projected_points[i] - ground_truth[i]);
        std::cout << "Point " << i
            << " | Projected: (" << projected_points[i].x << ", " << projected_points[i].y << ")"
            << " | GT: (" << ground_truth[i].x << ", " << ground_truth[i].y << ")"
            << " | Error: " << err << "\n";
        total_error += err;
    }
    std::cout << "Mean Reprojection Error: " << (total_error / projected_points.size()) << "\n";
}

vpHomogeneousMatrix run6DTPpipeline(const std::map<std::string, std::vector<vpImagePoint>>& qr_map, const vpTranslationVector& translation_mm, const vpRxyzVector& rotation_deg) {

    std::string qr_id = "001";

    // QRR 카메라 내부 파라미터
    vpCameraParameters cam_qrr(6187.0, 6187.0, 1024.0, 768.0);

    float qr_size = 30.0f;
    std::vector<vpPoint> object_points = {
        vpPoint(0, 0, 0),
        vpPoint(qr_size, 0, 0),
        vpPoint(qr_size, qr_size, 0),
        vpPoint(0, qr_size, 0)
    };

    if (qr_map.find(qr_id) == qr_map.end())
        throw std::runtime_error("QRR: QR ID " + qr_id + " not found.");

    // --- QRR 기준 QR의 Pose ---
    vpHomogeneousMatrix T_qrr_qr = computePose(qr_map.at(qr_id), object_points, cam_qrr);

    // --- BASE 기준 QR의 Pose (사용자 입력값 사용) ---
    vpHomogeneousMatrix T_base_qr;
    T_base_qr.buildFrom(
        translation_mm[0], translation_mm[1], translation_mm[2],
        vpMath::rad(rotation_deg[0]),
        vpMath::rad(rotation_deg[1]),
        vpMath::rad(rotation_deg[2])
    );

    // --- QRR 기준 BASE Pose 계산 ---
    vpHomogeneousMatrix T_qrr_base = compute6DTP(T_base_qr, T_qrr_qr);

    return T_qrr_base;
}

vpHomogeneousMatrix apply6DTP(const std::map<std::string, std::vector<vpImagePoint>>& qr_map, const vpHomogeneousMatrix& T_qrr_base) {

    std::string qr_id = "001";
    float qr_size = 30.0f;

    vpCameraParameters cam_qrr(6187.0, 6187.0, 1024.0, 768.0);

    std::vector<vpPoint> object_points = {
        vpPoint(0, 0, 0),
        vpPoint(qr_size, 0, 0),
        vpPoint(qr_size, qr_size, 0),
        vpPoint(0, qr_size, 0)
    };

    if (qr_map.find(qr_id) == qr_map.end())
        throw std::runtime_error("QR ID " + qr_id + " not found in second_tcp.txt");

    vpHomogeneousMatrix T_qrr_qr2 = computePose(qr_map.at(qr_id), object_points, cam_qrr);

    vpHomogeneousMatrix T_base_qr2 = T_qrr_base.inverse() * T_qrr_qr2;

    return T_base_qr2;
}


// --- 메인 ---
int main() {
    try {
        std::string flag;
        std::cout << "실행 모드를 선택하세요 [run cal tp, tp_test]: ";
        std::cin >> flag;

        // 카메라 간 리프로젝션 오차
        if (flag == "run") {
            std::string correction_matrix_path = "../output/correction_matrix_qrr1_to_qrr2.txt";
            vpHomogeneousMatrix correction_matrix = loadHomogeneousMatrix(correction_matrix_path);

            std::string qrr1_data = getQRDataFromServer("20.20.0.7", 9004);
            std::string qrr2_data = getQRDataFromServer("20.20.0.60", 9004);
            auto qr_map_qrr1 = parseQRData(qrr1_data);
            auto qr_map_qrr2 = parseQRData(qrr2_data);

            runReprojectionPipeline(qr_map_qrr1, qr_map_qrr2, correction_matrix);
        }
        // 카메라 간 보정행렬 정의
        else if (flag == "cal") {
            std::string qrr1_data = getQRDataFromServer("20.20.0.7", 9004);
            std::string qrr2_data = getQRDataFromServer("20.20.0.60", 9004);
            auto qr_map_qrr1 = parseQRData(qrr1_data);
            auto qr_map_qrr2 = parseQRData(qrr2_data);

            vpHomogeneousMatrix correction = computeCorrectionMatrix(qr_map_qrr1, qr_map_qrr2);
            // save and print
            std::string save_path = "../output/correction_matrix_qrr1_to_qrr2.txt";
            saveHomogeneousMatrix(correction, save_path);
            std::cout << "\n[Saved] → " << save_path << std::endl;
        }
        // 베이스-큐알 티피 정의
        else if (flag == "tp") {
            vpTranslationVector t_base_qr(243.36, 205.23, 220.00);  // [mm]
            vpRxyzVector r_base_qr(0, 0, 87.11);                    // [deg]

            std::string qrr1_data = getQRDataFromServer("20.20.0.7", 9004);
            auto qr_map = parseQRData(qrr1_data);

            vpHomogeneousMatrix T_qrr_base = run6DTPpipeline(qr_map, t_base_qr, r_base_qr);
            // save and print
            std::string save_path = "../output/6DTP_first.txt";
            saveHomogeneousMatrix(T_qrr_base, save_path);
            std::cout << "\n[Saved] → " << save_path << std::endl;

            printPose6D("QRR-BASE Pose (6D)", T_qrr_base);

        }
        // 베이스-큐알 티피 테스트
        else if (flag == "tp_test") {
            std::string qrr2_data = getQRDataFromServer("20.20.0.7", 9004);
            auto qr_map = parseQRData(qrr2_data);
            std::string tp_path = "../output/6DTP_first.txt";
            vpHomogeneousMatrix T_qrr_base = loadHomogeneousMatrix(tp_path);

            vpHomogeneousMatrix T_base_qr2 = apply6DTP(qr_map, T_qrr_base);

            // print T_base_qr2
            printPose6D("Second TCP location BASE-QR Pose (6D)", T_base_qr2);
        }
        else {
            std::cerr << "[입력 오류] 알 수 없는 플래그입니다: " << flag << "\n";
            std::cerr << "사용 가능한 플래그: run, cal\n";
            return 1;
        }
    }
    catch (const vpException& e) {
        std::cerr << "[ViSP Error] " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
    }

    return 0;
}
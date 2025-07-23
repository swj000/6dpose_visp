#include "sogang.h"
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

void printPose6D(const std::string& title, const vpHomogeneousMatrix& T) {
    vpTranslationVector t = T.getTranslationVector();
    vpRotationMatrix R = T.getRotationMatrix();
    vpThetaUVector rxyz;
    rxyz.buildFrom(R);

    double rx = vpMath::deg(rxyz[0]);
    double ry = vpMath::deg(rxyz[1]);
    double rz = vpMath::deg(rxyz[2]);

    std::cout << "\n[" << title << "]\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Tx: " << t[0] << " mm\n";
    std::cout << "Ty: " << t[1] << " mm\n";
    std::cout << "Tz: " << t[2] << " mm\n";
    std::cout << "Rx: " << rx << " deg\n";
    std::cout << "Ry: " << ry << " deg\n";
    std::cout << "Rz: " << rz << " deg\n";
}

void runReprojectionPipeline(const QR_CORNER_FORMAT& qrr1_map,
                             const QR_CORNER_FORMAT& qrr2_map,
                             const vpHomogeneousMatrix& correction) {
    float qr_size = 28.0f;

    // QRR1 intrinsics
    vpCameraParameters cam_qrr1(6187.0, 6187.0, 1024.0, 768.0);
    // QRR2 intrinsics
    cv::Mat K = (cv::Mat_<double>(3, 3) << 6670.0, 0, 1024.0,
                                           0, 6670.0, 768.0,
                                           0, 0, 1);
    cv::Mat distCoeffs;

    vpHomogeneousMatrix pose_qrr1 = computeQRPose(qrr1_map, cam_qrr1, qr_size);
    vpHomogeneousMatrix pose_qrr2 = correction * pose_qrr1;

    // ViSP pose to OpenCV rvec/tvec
    cv::Mat R_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_cv.at<double>(i, j) = pose_qrr2[i][j];

    cv::Mat rvec, tvec;
    cv::Rodrigues(R_cv, rvec);
    tvec = (cv::Mat_<double>(3, 1) << pose_qrr2[0][3], pose_qrr2[1][3], pose_qrr2[2][3]);

    // Print 6D pose
    std::cout << "\n[Pose in QRR2 Frame (Translation in mm, Rotation in deg)]\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Tx: " << tvec.at<double>(0) << " mm\n"
              << "Ty: " << tvec.at<double>(1) << " mm\n"
              << "Tz: " << tvec.at<double>(2) << " mm\n";

    cv::Mat rvec_deg = rvec * (180.0 / CV_PI);
    std::cout << "Rx: " << rvec_deg.at<double>(0) << " deg\n"
              << "Ry: " << rvec_deg.at<double>(1) << " deg\n"
              << "Rz: " << rvec_deg.at<double>(2) << " deg\n";

    // Define object points
    std::vector<cv::Point3f> object_cv = {
        {0, 0, 0},
        {qr_size, 0, 0},
        {qr_size, qr_size, 0},
        {0, qr_size, 0}
    };

    // Reprojection
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_cv, rvec, tvec, K, distCoeffs, projected_points);

    std::vector<cv::Point2f> ground_truth;
    for (int i = 0; i < 4; ++i)
        ground_truth.emplace_back(qrr2_map.x[i], qrr2_map.y[i]);

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



int main() {
    try {
        std::string flag;
        std::cout << "실행 모드를 선택하세요 [cal, run, tp, tptest, testcode]: ";
        std::cin >> flag;

        if (flag == "cal") {
            std::string targetQRID = "001";
            std::string qrr1_data = getQRDataFromServer("20.20.0.7", 9004);
            std::string qrr2_data = getQRDataFromServer("20.20.0.60", 9004);
            QR_CORNER_FORMAT qr_map_qrr1 = parseQRData(qrr1_data, targetQRID);
            QR_CORNER_FORMAT qr_map_qrr2 = parseQRData(qrr2_data, targetQRID);

            vpCameraParameters cam_qrr1(6187.0, 6187.0, 1024.0, 768.0);
            vpCameraParameters cam_qrr2(6670.0, 6670.0, 1024.0, 768.0);
            float qr_size = 30.0;

            vpHomogeneousMatrix qrr1_pose = computeQRPose(qr_map_qrr1, cam_qrr1, qr_size);
            vpHomogeneousMatrix qrr2_pose = computeQRPose(qr_map_qrr2, cam_qrr2, qr_size);

            saveHomogeneousMatrix(qrr1_pose, "../output/qrr1_boydqr_pose.txt");
            saveHomogeneousMatrix(qrr2_pose, "../output/qrr2_boydqr_pose.txt");

            std::cout << "[Saved] Pose files.\n";
        }
        else if (flag == "run") {
            vpHomogeneousMatrix qrr1_pose = loadHomogeneousMatrix("../output/qrr1_boydqr_pose.txt");
            vpHomogeneousMatrix qrr2_pose = loadHomogeneousMatrix("../output/qrr2_boydqr_pose.txt");
            vpHomogeneousMatrix correction_matrix = computeCorrectionMatrix(qrr1_pose, qrr2_pose);

            std::string targetQRID = "1807";
            QR_CORNER_FORMAT qr_map_qrr1 = parseQRData(getQRDataFromServer("20.20.0.7", 9004), targetQRID);
            QR_CORNER_FORMAT qr_map_qrr2 = parseQRData(getQRDataFromServer("20.20.0.60", 9004), targetQRID);

            runReprojectionPipeline(qr_map_qrr1, qr_map_qrr2, correction_matrix);
        }
        else if (flag == "tp") {
            vpTranslationVector t_base_qr(243.36, 205.23, 220.00);
            vpRxyzVector r_base_qr(0, 0, 87.11);
            std::string targetQRID = "001";
            QR_CORNER_FORMAT qr_map = parseQRData(getQRDataFromServer("20.20.0.7", 9004), targetQRID);

            vpHomogeneousMatrix T_qrr_base = compute6DTP(qr_map, t_base_qr, r_base_qr);
            saveHomogeneousMatrix(T_qrr_base, "../output/6DTP_first.txt");

            printPose6D("QRR-BASE Pose (6D)", T_qrr_base);
        }
        else if (flag == "tptest") {
            std::string targetQRID = "001";
            QR_CORNER_FORMAT qr_map = parseQRData(getQRDataFromServer("20.20.0.7", 9004), targetQRID);
            vpHomogeneousMatrix T_qrr_base = loadHomogeneousMatrix("../output/6DTP_first.txt");

            vpHomogeneousMatrix T_base_qr2 = apply6DTP(qr_map, T_qrr_base);
            printPose6D("Second TCP location BASE-QR Pose (6D)", T_base_qr2);
        }
        // afafaafbdagaed
        else if (flag == "testcode") {
            std::vector<std::string> qr_ids = {"001", "002"};

            vpCameraParameters cam_qrr1(6187.0, 6187.0, 1024.0, 768.0);
            vpCameraParameters cam_qrr2(6670.0, 6670.0, 1024.0, 768.0);
            float qr_size = 30.0;

            std::vector<vpHomogeneousMatrix> correction_matrices;

            for (const auto& targetQRID : qr_ids) {
                std::string qrr1data = getQRDataFromTextFile("../points/qrr1_zig.txt");
                std::string qrr2data = getQRDataFromTextFile("../points/qrr2_zig.txt");
                QR_CORNER_FORMAT qr_map_qrr1 = parseQRData(qrr1data, targetQRID);
                QR_CORNER_FORMAT qr_map_qrr2 = parseQRData(qrr2data, targetQRID);

                vpHomogeneousMatrix qrr1_pose = computeQRPose(qr_map_qrr1, cam_qrr1, qr_size);
                vpHomogeneousMatrix qrr2_pose = computeQRPose(qr_map_qrr2, cam_qrr1, qr_size);

                vpHomogeneousMatrix correction_matrix = computeCorrectionMatrix(qrr1_pose, qrr2_pose);

                correction_matrices.push_back(correction_matrix);
            }

            vpMatrix mat_sum(4, 4, 0.0);
            for (const auto& mat : correction_matrices) {
                vpMatrix M = mat;
                mat_sum += M;
            }
            mat_sum /= correction_matrices.size();

            // 평균 행렬에서 R, t 분리
            vpRotationMatrix R_avg;
            vpTranslationVector t_avg;

            for (unsigned int i = 0; i < 3; ++i) {
                t_avg[i] = mat_sum[i][3];
                for (unsigned int j = 0; j < 3; ++j) {
                    R_avg[i][j] = mat_sum[i][j];
                }
            }

            vpHomogeneousMatrix result(t_avg, R_avg);

            std::cout << "Average Correction Matrix:\n" << result << std::endl;

            saveHomogeneousMatrix(result, "../output/correction_matrix_ms.txt");

            // ---------------------------------- //
            std::string qrr1data = getQRDataFromTextFile("../points/qrr1_station.txt");
            std::string qrr2data = getQRDataFromTextFile("../points/qrr2_station.txt");
            QR_CORNER_FORMAT qr_map_qrr1 = parseQRData(qrr1data, "1807");
            QR_CORNER_FORMAT qr_map_qrr2 = parseQRData(qrr2data, "1807");

            runReprojectionPipeline(qr_map_qrr1, qr_map_qrr2, result);

        }
        else {
            std::cerr << "[입력 오류] 알 수 없는 플래그입니다: " << flag << "\n";
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

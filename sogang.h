#ifndef SOGANG_H
#define SOGANG_H

#include <string>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpRxyzVector.h>

struct QR_CORNER_FORMAT {
    unsigned int id;
    int x[4];
    int y[4];
};

QR_CORNER_FORMAT parseQRData(const std::string& text, const std::string& targetQRID);
std::string getQRDataFromServer(const std::string& ip, int port, const std::string& command = "LON\r\n");

void saveHomogeneousMatrix(const vpHomogeneousMatrix& mat, const std::string& filepath);
vpHomogeneousMatrix loadHomogeneousMatrix(const std::string& filepath);

vpHomogeneousMatrix computeQRPose(const QR_CORNER_FORMAT& qr, const vpCameraParameters& cam, float qr_size);
vpHomogeneousMatrix computeCorrectionMatrix(const vpHomogeneousMatrix& T_qrr1, const vpHomogeneousMatrix& T_qrr2);

vpHomogeneousMatrix compute6DTP(const QR_CORNER_FORMAT& qr_map, const vpTranslationVector& translation_mm, const vpRxyzVector& rotation_deg);
vpHomogeneousMatrix apply6DTP(const QR_CORNER_FORMAT& qr_map, const vpHomogeneousMatrix& T_qrr_base);

std::string getQRDataFromTextFile(const std::string& filePath);


#endif // SOGANG_H

#ifndef QRDETECTOR_H
#define QRDETECTOR_H

#include <vector>
#include <string>

#include "opencv2/objdetect/objdetect.hpp"

#ifdef WITH_ZBAR
#include "zbar.h"
#endif

#ifdef WITH_DEBUG_OUTPUT
#define DEBUG(x) x
#else
#define DEBUG(x)
#endif

class QRDescriptor {
    friend class QRDetector;

public:
    cv::Rect boundingRect() const;
    bool hasAlignment() const;

    std::vector<cv::Rect> fips;
    cv::Rect alignment;
    cv::Size size;
    int eps;

private:
    void addFip(int x = 0, int y = 0, int w = 0);
};

struct QROutput {
    cv::Mat pic;
#ifdef WITH_QR_RECOGNITION
    std::vector<std::vector<bool> > data;
#endif
    std::string content;
    QRDescriptor qrd;

    double xAngle, yAngle, zAngle;
    double distance, shift;
    bool detected;
};

class QRDetector {
public:
    QRDetector(const std::string &fipCascade, const std::string &markerCascade, const std::string &cameraMatrixData, float realQRSize);
    QRDetector(const std::string &fipCascade, const std::string &markerCascade);

    bool isReady() const { return ready; }

    std::vector<std::vector<QRDescriptor> > detectQRs(const cv::Mat &img);
    QROutput processQRGroup(std::vector<QRDescriptor> &qrg, const cv::Mat &img, bool recognize = true);

private:
    void postprocessQRDescriptor(QRDescriptor &qrd) const;
    cv::Rect detectAlignmentMarker(const QRDescriptor &qrd, const cv::Mat &img);
    QROutput processQR(const QRDescriptor &qrd, const cv::Mat &img);

    bool ready;
    float qrSize;
    cv::CascadeClassifier fipDetector, markerDetector;
    cv::Mat cameraMatrix, cameraMatrixInv;
#ifdef WITH_ZBAR
    zbar::ImageScanner qrScanner;
#endif
};

#endif // QRDETECTOR_H

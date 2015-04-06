#include "qrdetector.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <math.h>
#include <iostream>

#define CV_CENTER(rect) cv::Point2i(rect.x + rect.width / 2, rect.y + rect.height / 2)

static const int qrVersions[] = { 21, 25, 29, 33, 57, 117, 177 };
static const int qrVersionsCount = (sizeof(qrVersions) / sizeof(int));

//-----------------------------------------------------------------------------

QRDetector::QRDetector(const std::string &fipCascade, const std::string &markerCascade, const std::string &cameraMatrixData, float realQRSize) : ready(false), qrSize(realQRSize) {
    ready = fipDetector.load(fipCascade) && markerDetector.load(markerCascade);
    if(!ready) return;

    cv::FileStorage fs(cameraMatrixData, cv::FileStorage::READ);
    ready = fs.isOpened();
    if(!ready) return;

    fs["CameraMatrix"] >> cameraMatrix;
    ready = !cameraMatrix.empty();
    if(!ready) return;

    cameraMatrixInv = cameraMatrix.inv();

#ifdef WITH_ZBAR
    qrScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    qrScanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
#endif
}

QRDetector::QRDetector(const std::string &fipCascade, const std::string &markerCascade) : ready(false), qrSize(1.0) {
    ready = fipDetector.load(fipCascade) && markerDetector.load(markerCascade);
    if(!ready) return;

    cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
    cameraMatrixInv = cameraMatrix.inv();

#ifdef WITH_ZBAR
    qrScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    qrScanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
#endif
}

//-----------------------------------------------------------------------------

struct fipv {
    fipv(int x = 0, int y = 0, int size = 0) : x(x), y(y), size(size), width(size / 7) {}

    int x;
    int y;
    int size;
    int width;
};

struct QRDesriptorEpsComparator {
    bool operator ()(const QRDescriptor &d1, const QRDescriptor &d2) {
        return d1.eps > d2.eps;
    }
};

std::vector<std::vector<QRDescriptor> > QRDetector::detectQRs(const cv::Mat &img) {
    std::vector<cv::Rect> rects;
    std::vector<std::vector<QRDescriptor> > groups;

    fipDetector.detectMultiScale(img, rects, 1.1, 1, 0, cv::Size(7, 7), cv::Size(128, 128));
    if(rects.size() < 3) return groups;

    std::vector<fipv> v;
    for(std::vector<cv::Rect>::const_iterator r = rects.begin(); r != rects.end(); ++r) {
        v.push_back(fipv(r->x + r->width / 2, r->y + r->height / 2, (r->width + r->height) / 2));
    }

    std::vector<std::vector<char> > adj;
    int vc = 1;
    for(std::vector<fipv>::iterator v1 = v.begin(); v1 != v.end(); ++v1, ++vc) {
        adj.push_back(std::vector<char>(vc, 0));
        for(std::vector<fipv>::iterator v2 = v1 + 1; v2 != v.end(); ++v2) {
            int size = (v1->size + v2->size) / 2;
            if(abs(v1->size - v2->size) < 0.25 * size
                    && abs(v1->x - v2->x) < 19 * size
                    && abs(v1->y - v2->y) < 19 * size
                    && (abs(v1->x - v2->x) > 1.6 * size
                    || abs(v1->y - v2->y) > 1.6 * size)
                    ) {
//                qDebug() << "mind:" << ((v1->size + v2->size) / 2) << "y:" << qAbs(v1->y - v2->y) << "x:" << qAbs(v1->x - v2->x);
                adj.back().push_back(1);
            } else {
                adj.back().push_back(0);
            }
        }
    }

    // search for 3-cycles in fip graph
    std::vector<QRDescriptor> qrs;
    for(size_t i = 0; i < v.size(); ++i) {
        for(size_t j = i + 1; j < v.size(); ++j) {
            if(!adj[i][j]) continue;
            for(size_t k = j + 1; k < v.size(); ++k) {
                if(adj[j][k] && adj[i][k]) {
                    QRDescriptor d;
                    d.addFip(v[i].x, v[i].y, v[i].size);
                    d.addFip(v[j].x, v[j].y, v[j].size);
                    d.addFip(v[k].x, v[k].y, v[k].size);
                    qrs.push_back(d);
                }
            }
        }
    }

    size_t ssize = qrs.size();
    for(std::vector<QRDescriptor>::iterator d = qrs.begin(); d != qrs.end();) {
        int dist[3];
        int maxd = -1;
        for(int i = 0; i < 3; ++i) {
            cv::Point2i p1 = CV_CENTER(d->fips[i]), p2 = CV_CENTER(d->fips[(i + 1) % 3]);
            dist[i] = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
            if(maxd == -1 || dist[maxd] < dist[i]) maxd = i;
        }
        //right triangle check; 20% is enough for geometric distortion
        int d1 = (maxd + 1) % 3;
        int d2 = (maxd + 2) % 3;
        d->eps = fabs(dist[d1] + dist[d2] - dist[maxd]);
        if(d->eps > 0.2 * dist[maxd] || fabs(dist[d1] - dist[d2]) > 0.1 * (dist[d1] + dist[d2])) {
            d = qrs.erase(d);
        } else {
            postprocessQRDescriptor(*d);
            ++d;
        }
    }

    DEBUG(if(ssize - qrs.size() > 0) std::cout << (ssize - qrs.size()) << "candidates removed due to right triangle constraint" << std::endl);

    if(qrs.empty()) return groups;

    // now we should group destriptors with intersections
    // only 1 descriptor in such group is valid
    // THAT'S BUGGY
    if(qrs.size() > 1) {
        int g = 0;
        std::map<std::pair<int, int>, int> groupMap;
        for(std::vector<QRDescriptor>::iterator d = qrs.begin(); d != qrs.end(); ++d) {
            int cgroup = -1;
            for(int i = 0; i < 3; ++i) {
                std::map<std::pair<int, int>, int>::iterator f = groupMap.find(std::make_pair(d->fips[i].x, d->fips[i].y));
                if(f != groupMap.end()) {
                    cgroup = f->second;
                    break;
                }
            }
            int addgroup = cgroup == -1 ? g++ : cgroup;
            for(int i = 0; i < 3; ++i) groupMap[std::make_pair(d->fips[i].x, d->fips[i].y)] = addgroup;
            if(cgroup == -1) {
                groups.push_back(std::vector<QRDescriptor>());
                groups.back().push_back(*d);
            } else {
                groups[cgroup].push_back(*d);
            }
        }
    } else {
        groups.push_back(qrs);
    }

    // next we should sort fips in groups by max rightness
    for(std::vector<std::vector<QRDescriptor> >::iterator g = groups.begin(); g != groups.end(); ++g) {
        std::sort(g->begin(), g->end(), QRDesriptorEpsComparator());
    }

    return groups;
}

//-----------------------------------------------------------------------------

QROutput QRDetector::processQRGroup(std::vector<QRDescriptor> &qrg, const cv::Mat &img, bool recognize) {
    QROutput res;
    res.detected = false;
    int cnt = 0;
    for(std::vector<QRDescriptor>::iterator i = qrg.begin(); i != qrg.end(); ++i, ++cnt) {
        i->alignment = detectAlignmentMarker(*i, img);
        if(!i->hasAlignment()) continue;
        if(recognize) {
            res = processQR(*i, img);
            if(!res.content.empty()) {
                res.detected = true;
                res.qrd = *i;
                DEBUG(std::cout << "matched at" << cnt << "try" << std::endl);
                return res;
            }
        } else if(i->hasAlignment()) {
            res.detected = true;
            res.qrd = *i;
            return res;
        }
    }
    return res;
}

//-----------------------------------------------------------------------------

#define CV_DIST2(p1, p2) ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

cv::Rect QRDetector::detectAlignmentMarker(const QRDescriptor &qrd, const cv::Mat &img) {
    cv::Rect qrr = qrd.boundingRect();

    std::vector<cv::Rect> aligns;
    markerDetector.detectMultiScale(img(qrr), aligns, 1.1, 1, 0, cv::Size(5, 5), cv::Size(128, 128));

    int dw = 0, dh = 0;
    for(std::vector<cv::Rect>::const_iterator f = qrd.fips.begin(); f != qrd.fips.end(); ++f) {
        dw += f->width;
        dh += f->height;
    }
    dw /= qrd.fips.size();
    dh /= qrd.fips.size();

    // remove bad matches
    cv::Rect bestMatch;
    double bestEps = 0;
    for(std::vector<cv::Rect>::iterator a = aligns.begin(); a != aligns.end(); ++a) {
        if(a->width < 0.4 * dw || a->width > dw || a->height < 0.4 * dh || a->height > dh) continue;

        cv::Rect marker(a->x + qrr.x, a->y + qrr.y, a->width, a->height);
        bool intersects = false;
        for(std::vector<cv::Rect>::const_iterator f = qrd.fips.begin(); f != qrd.fips.end(); ++f) {
            if((*f & marker).area()) {
                intersects = true;
                break;
            }
        }

        if(intersects) continue;
        else {
            cv::Point2i mc = CV_CENTER(marker);
            int dist1 = CV_DIST2(mc, qrd.fips[1]);
            int dist2 = CV_DIST2(mc, qrd.fips[2]);
            int dist3 = CV_DIST2(CV_CENTER(qrd.fips[1]), CV_CENTER(qrd.fips[2]));
            int d_eps = fabs(dist3 - dist1 - dist2);
            if(fabs(dist1 - dist2) > 0.1 * (dist1 + dist2) || d_eps > 0.2 * dist3) {
//                std::cout << "marker removed due to right triangle constraint: " << d_eps << std::endl;
                continue;
            } else if(!bestMatch.area() || d_eps < bestEps) {
                bestMatch = marker;
                bestEps = d_eps;
            }
        }
    }

    DEBUG(if(bestMatch.area()) std::cout << "best match eps: " << bestEps << std::endl);
    return bestMatch;
}

//-----------------------------------------------------------------------------

#ifdef WITH_QR_RECOGNIZE
static inline int regionSum(const cv::Mat &img, const cv::Rect &rect) {
    return img.at<int>(rect.br().x, rect.br().y) - img.at<int>(rect.tl().x, rect.br().y) - img.at<int>(rect.br().x, rect.tl().y) + img.at<int>(rect.tl().x, rect.tl().y);
}
#endif

QROutput QRDetector::processQR(const QRDescriptor &qrd, const cv::Mat &img) {
    QROutput out;

    int lineWidth = 8;
    float shift = lineWidth * 3.5; // left fip center pos on deskewed image
    cv::Size dstSize(lineWidth * qrd.size.width, lineWidth * qrd.size.height);
    cv::Point2f dstPos[] = { cv::Point2f(shift, shift),
                             cv::Point2f(dstSize.width - shift, shift),
                             cv::Point2f(shift, dstSize.height - shift),
                             cv::Point2f(dstSize.width - 6.5 * lineWidth, dstSize.height - 6.5 * lineWidth)
                           };

    cv::Point2f srcPos[4] = { CV_CENTER(qrd.fips[0]),
                              CV_CENTER(qrd.fips[1]),
                              CV_CENTER(qrd.fips[2]),
                              CV_CENTER(qrd.alignment)
                            };

    cv::Mat H = cv::getPerspectiveTransform(srcPos, dstPos);

    //-------------------------------------------------------------------------

    cv::Mat H1 = cameraMatrixInv * H;
    cv::Mat pose = cv::Mat::eye(3, 4, CV_64FC1);
    float tnorm = (cv::norm(H1.col(0)) + cv::norm(H1.col(1))) / 2.0f;

    cv::normalize(H1.col(0), pose.col(0));
    cv::normalize(H1.col(1), pose.col(1));
    pose.col(0).cross(pose.col(1)).copyTo(pose.col(2));
    pose.col(3) = H1.col(2) / tnorm;

    double r32 = pose.at<double>(2, 1);
    double r33 = pose.at<double>(2, 2);

    out.xAngle = atan2(r32, r33);
    out.yAngle = atan2(-pose.at<double>(2, 0), sqrt(r32 * r32 + r33 * r33));
    out.zAngle = atan2(pose.at<double>(1, 0), pose.at<double>(0, 0));

    cv::Rect br = qrd.boundingRect();
    out.distance = cameraMatrix.at<double>(1,1) * qrSize / (float)br.height;
    out.shift = (br.x + br.width / 2 - cameraMatrix.at<double>(0, 2)) * qrSize / (float)br.width;

    //-------------------------------------------------------------------------

    cv::Mat res(dstSize.width, dstSize.height, img.type());
    cv::warpPerspective(img, res, H, dstSize);

    double minVal, maxVal;
    cv::minMaxIdx(res, &minVal, &maxVal);
    cv::threshold(res, res, (minVal + maxVal) / 2, 255, cv::THRESH_BINARY);

    //-------------------------------------------------------------------------

#ifdef WITH_ZBAR
    // qrScanner crashes the program. WTF???
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    zbar::Image zimg(res.cols, res.rows, "GREY", res.data, res.rows * res.cols * res.elemSize());
    int scan_res = scanner.scan(zimg);
//    int scan_res = qrScanner.scan(zimg);
    if(scan_res > 0) {
//        const zbar::SymbolSet &syms = qrScanner.get_results();
        const zbar::SymbolSet &syms = scanner.get_results();
        for(zbar::SymbolIterator i = syms.symbol_begin(); i != syms.symbol_end(); ++i) out.content += "symbol (" + i->get_type_name() + "): " + i->get_data() + "\n";
    }
    zimg.set_data(NULL, 0);
#endif

    //-------------------------------------------------------------------------

#ifdef WITH_QR_RECOGNITION
    cv::Mat ires(dstSize.width + 1, dstSize.height + 1, CV_32F);
//    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(lineWidth / 8, lineWidth / 8));
//    cv::dilate(res, res, kernel);
    cv::integral(res, ires);

    int yshift = 0, xshift = 0;
    for(int r = 0; r < res.rows; ++r) {
        if(res.at<uchar>(r, 2 * lineWidth) == 0) {
            yshift = r;
            break;
        }
    }
    for(int c = 0; c < res.cols; ++c) {
        if(res.at<uchar>(2 * lineWidth, c) == 0) {
            xshift = c;
            break;
        }
    }

    for(int r = 0; r < qrd.size.height; ++r) {
        out.data.push_back(std::vector<bool>(qrd.size.width, 0));

        int rdif = res.rows - yshift - r * lineWidth;
        int rsize = rdif < lineWidth ? rdif : lineWidth;
        for(int c = 0; c < qrd.size.width; ++c) {
            int cdif = res.cols - xshift - c * lineWidth;
            int csize = cdif < lineWidth ? cdif : lineWidth;
            float v = regionSum(ires, cv::Rect(r * lineWidth + yshift, c * lineWidth + xshift, rsize, csize)) / (255.0 * rsize * csize);
            out.data.back()[c] = v > 0.50;
        }
    }
#endif

    out.pic = res;
    return out;
}

//-----------------------------------------------------------------------------

static inline float rectDistance(const cv::Rect &r1, const cv::Rect &r2) {
    cv::Point2i c1 = CV_CENTER(r1);
    cv::Point2i c2 = CV_CENTER(r2);
    return sqrtf((c1.x - c2.x) * (c1.x - c2.x) + (c1.y - c2.y) * (c1.y - c2.y));
}

void QRDetector::postprocessQRDescriptor(QRDescriptor &qrd) const {
    std::vector<cv::Rect> &fips = qrd.fips;

    float maxd = 0;
    size_t maxi = 0;
    for(size_t i = 0; i < fips.size(); ++i) {
        float d = rectDistance(fips[i], fips[(i + 1) % fips.size()]);
        if(d > maxd) {
            maxd = d;
            maxi = i;
        }
    }

    cv::Rect topLeft, topRight, bottomLeft;
    topLeft = fips[(maxi + 2) % fips.size()];
    if(fips[maxi].y < fips[(maxi + 1) % fips.size()].y) {
        topRight = fips[maxi];
        bottomLeft = fips[(maxi + 1) % fips.size()];
    } else {
        topRight = fips[(maxi + 1) % fips.size()];
        bottomLeft = fips[maxi];
    }

    fips.clear();
    fips.push_back(topLeft);
    fips.push_back(topRight);
    fips.push_back(bottomLeft);

    // pixel size of the detected qr
    int pixY = (CV_CENTER(bottomLeft).y - CV_CENTER(topLeft).y) * 14 / (bottomLeft.height + topLeft.height) + 9;    //detected fip is slightly bigger than real
    int pixX = (CV_CENTER(topRight).x - CV_CENTER(topLeft).x) * 14 / (topRight.width + topRight.width) + 9;         //so we should add 2 to it's pixel size
    float pix = (pixX + pixY) / 2.0;

    float minD = fabs(qrVersions[0] - pix);
    int minP = 0;
    for(int i = 1; i < qrVersionsCount; ++i) {
        float d = fabs(qrVersions[i] - pix);
        if(d <= minD) {
            minD = d;
            minP = i;
        }
    }

    qrd.size = cv::Size(qrVersions[minP], qrVersions[minP]);
}

//-----------------------------------------------------------------------------

void QRDescriptor::addFip(int x, int y, int w) {
    fips.push_back(cv::Rect(x - w / 2, y - w / 2, w, w));
}

cv::Rect QRDescriptor::boundingRect() const {
    cv::Point2i topLeft = fips.front().tl(), bottomRight = fips.front().br();
    for(std::vector<cv::Rect>::const_iterator r = fips.begin() + 1; r != fips.end(); ++r) {
        if(r->y < topLeft.y) topLeft.y = r->y;
        if(r->x < topLeft.x) topLeft.x = r->x;
        if(r->br().y > bottomRight.y) bottomRight.y = r->br().y;
        if(r->br().x > bottomRight.x) bottomRight.x = r->br().x;
    }
    return cv::Rect(topLeft, bottomRight);
}

bool QRDescriptor::hasAlignment() const {
    return alignment.area() != 0;
}

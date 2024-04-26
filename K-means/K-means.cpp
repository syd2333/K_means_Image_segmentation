#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define _CRT_SECURE_NO_WARNINGS

#include "stb_image_write.h"
#include "stb_image.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <map>

using namespace std;

struct Point {
    int x;  // 像素的x坐标
    int y;  // 像素的y坐标
    int r;  // 红色通道值
    int g;  // 绿色通道值
    int b;  // 蓝色通道值
    int cluster;

    Point(int x, int y, int r, int g, int b) : x(x), y(y), r(r), g(g), b(b), cluster(-1) {}
};

void PrintPoints(const vector<Point>& points) {
    for (int i = 0; i < points.size(); i++) {
        cout << "Point " << i << ": (" << points[i].x << ", " << points[i].y << "), RGB: ("
            << points[i].r << ", " << points[i].g << ", " << points[i].b << "), Cluster: "
            << points[i].cluster << endl;
    }
}

tuple<vector<Point>, int, int> readImageData(const string& filename) {
    cout << "读取 " << filename << " 中..." << endl;
    vector<Point> points;
    int width, height, channels;
    unsigned char* img = stbi_load(filename.c_str(), &width, &height, &channels, 0);
    if (img == NULL) {
        cout << "Error in loading the image" << endl;
        return make_tuple(points, 0, 0);
    }
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = (i * width + j) * channels;
            int r = img[index];
            int g = img[index + 1];
            int b = img[index + 2];
            points.emplace_back(j, i, r, g, b);
        }
    }
    stbi_image_free(img);

    cout << filename << " 读取完成" << endl;

    return make_tuple(points, width, height);
}

double getDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dR = p1.r - p2.r;
    double dG = p1.g - p2.g;
    double dB = p1.b - p2.b;
    return sqrt(dx * dx + dy * dy + 20 * dR * dR + 20 * dG * dG + 20 * dB * dB);  //提高了RGB对于距离的权重
}

vector<Point> initCentroids(const vector<Point>& points, int K) {
    vector<Point> centroids;
    srand(time(0));
    for (int i = 0; i < K; i++) {
        int randomIndex = rand() % points.size();
        Point centroid = points[randomIndex];
        centroid.cluster = i;
        centroids.push_back(centroid);
    }
    return centroids;
}

bool assignClusters(vector<Point>& points, vector<Point>& centroids) {
    bool changed = false;
    for (Point& point : points) {
        double minDistance = getDistance(point, centroids[0]);
        int newCluster = 0;
        for (int i = 1; i < centroids.size(); i++) {
            double distance = getDistance(point, centroids[i]);
            if (distance < minDistance) {
                minDistance = distance;
                newCluster = i;
            }
        }
        if (point.cluster != newCluster) {
            changed = true;
            point.cluster = newCluster;
        }
    }
    return changed;
}

void updateCentroids(vector<Point>& centroids, vector<Point>& points) {
    vector<int> counts(centroids.size(), 0);
    vector<int> sumX(centroids.size(), 0);
    vector<int> sumY(centroids.size(), 0);
    vector<int> sumR(centroids.size(), 0);
    vector<int> sumG(centroids.size(), 0);
    vector<int> sumB(centroids.size(), 0);

    for (const Point& point : points) {
        int cluster = point.cluster;
        counts[cluster]++;
        sumX[cluster] += point.x;
        sumY[cluster] += point.y;
        sumR[cluster] += point.r;
        sumG[cluster] += point.g;
        sumB[cluster] += point.b;
    }

    for (int i = 0; i < centroids.size(); i++) {
        centroids[i].x = counts[i] ? sumX[i] / counts[i] : sumX[i];
        centroids[i].y = counts[i] ? sumY[i] / counts[i] : sumY[i];
        centroids[i].r = counts[i] ? sumR[i] / counts[i] : sumR[i];
        centroids[i].g = counts[i] ? sumG[i] / counts[i] : sumG[i];
        centroids[i].b = counts[i] ? sumB[i] / counts[i] : sumB[i];
    }
}

vector<Point> K_means(vector<Point>& points, int K, int maxIterations) {
    vector<Point> centroids = initCentroids(points, K);

    cout << "分割中..." << endl;

    int iter = 0;
    bool changed = true;
    while (changed && iter < maxIterations) {
        changed = assignClusters(points, centroids);
        updateCentroids(centroids, points);
        iter++;

        cout << iter << "/" << maxIterations << " ";
    }

    if (!changed)
        cout << maxIterations << "/" << maxIterations;

    cout << "\n分割完成" << endl;
    return centroids;
}

void generateImage(const vector<Point>& points, const vector<Point>& centroids, int width, int height, const string& outputFilename) {
    cout << "生成 " << outputFilename << " 中..." << endl;
    int bytesPerPixel = 3; // RGB 格式每个像素占 3 字节
    int stride = width * bytesPerPixel;
    vector<unsigned char> data(stride * height);

    for (const Point& point : points) {
        int index = (point.y * width + point.x) * bytesPerPixel;
        data[index] = centroids[point.cluster].r;
        data[index + 1] = centroids[point.cluster].g;
        data[index + 2] = centroids[point.cluster].b;
    }

    stbi_write_png(outputFilename.c_str(), width, height, bytesPerPixel, data.data(), stride);
    cout << "已生成 " << outputFilename << endl;
}

int main() {
    srand(static_cast<unsigned int>(time(0)));

    string filename, outputFilename;
    cout << "输入图片名（需要png格式，例如:scene.png）:";
    cin >> filename;
    vector<Point> points;
    int width, height;
    tie(points, width, height) = readImageData(filename);
    if (points.empty()) {
        return 1;
    }

    int K, Max;
    cout << "输入图片分割后的块数（越大越准确，但是迭代速度会变慢，推荐5-30）:";
    cin >> K;
    cout << "输入最高迭代次数（防止运行时间过长，推荐150以上）:";
    cin >> Max;
    vector<Point> centroids = K_means(points, K, Max);

    cout << "输入分块后的图片名（需要png格式，例如:output.png）:";
    cin >> outputFilename;
    
    generateImage(points, centroids, width, height, outputFilename);

    system("pause");

    return 0;
}
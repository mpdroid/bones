#include "clusterizer.h"
using namespace clusterizer;

Clusterizer::Clusterizer(cilantro::VectorSet3f *cilantroPoints, cilantro::VectorSet3f *cilantroColors)
{
    this->cloud.points = *cilantroPoints;
    this->cloud.colors = *cilantroColors;
}

Clusterizer::~Clusterizer()
{
    this->cloud.clear();
}

vector<BoundingCube> Clusterizer::Clusterize(cilantro::PointCloud3f *segmentedCloud)
{
    vector<BoundingCube> boundingCubes;

    cilantro::RadiusNeighborhoodSpecification<float> nh(0.02f * 0.02f);
    cilantro::ConnectedComponentExtraction3f<> cce(this->cloud.points);
    cilantro::PointsColorsProximityEvaluator<float, 3> ev(cloud.colors, 0.1f, 0.05f);

    cce.segment(nh, ev, 200, this->cloud.size());
    size_t num_labels = cce.getNumberOfClusters();
    const auto &labels = cce.getPointToClusterIndexMap();

    cilantro::VectorSet3f color_map(3, num_labels + 1);
    for (size_t i = 0; i < num_labels; i++)
    {
        color_map.col(i) = Eigen::Vector3f::Random().cwiseAbs();
    }
    color_map.col(num_labels).setZero();
    cilantro::VectorSet3f colors(3, labels.size());
    for (size_t i = 0; i < colors.cols(); i++)
    {
        colors.col(i) = color_map.col(labels[i]);
    }

    (*segmentedCloud) = cilantro::PointCloud3f(cloud.points, cloud.normals, colors);

    const auto &cpi = cce.getClusterToPointIndicesMap();
    for (size_t i = 0; i < cpi.size() - 1; i++)
    {
        float maxx = -INFINITY;
        float maxy = -INFINITY;
        float maxz = -INFINITY;
        float minx = INFINITY;
        float miny = INFINITY;
        float minz = INFINITY;
        Eigen::MatrixXf points(cpi[i].size(), 3);
        Eigen::MatrixXi pointColors(cpi[i].size(), 4);
        for (size_t j = 0; j < cpi[i].size(); j++)
        {
            int p = (int)cpi[i][j];
            float x = this->cloud.points(0, p);
            float y = this->cloud.points(1, p);
            float z = this->cloud.points(2, p);
            points.row(j) << x * 1000.f, y * 1000.f, z * 1000.f;

            pointColors.row(j) << (int)(this->cloud.colors(0, p) * 255.f),
                (int)(this->cloud.colors(1, p) * 255.f),
                (int)(this->cloud.colors(2, p) * 255.f),
                255;
        }

        Eigen::Vector3f ftl = points.colwise().minCoeff();
        Eigen::Vector3f bbr = points.colwise().maxCoeff();
        Vector3f ftr, fbr, fbl, btl, btr, bbl;
        ftr << bbr[0], ftl[1], ftl[2];
        fbr << bbr[0], bbr[1], ftl[2];
        fbl << ftl[0], bbr[1], ftl[2];
        btl << ftl[0], ftl[1], bbr[2];
        btr << bbr[0], ftl[1], bbr[2];
        bbl << ftl[0], bbr[1], bbr[2];

        BoundingCube cube;
        cube.u = ftl;
        cube.v = bbr;
        cube.ftl = ftl;
        cube.ftr = ftr;
        cube.fbr = fbr;
        cube.fbl = fbl;
        cube.btl = btl;
        cube.btr = btr;
        cube.bbr = bbr;
        cube.bbl = bbl;
        cube.u_k4 = {ftl[0], ftl[1], ftl[2]};
        cube.v_k4 = {bbr[0], bbr[1], bbr[2]};
        cube.u2d_k4 = {0.f, 0.f};
        cube.v2d_k4 = {0.f, 0.f};
        auto color = color_map.col(i);
        cube.color = {color[0] * 255.f, color[1] * 255.f, color[2] * 255.f};
        cube.label = i;
        cube.points = points;
        cube.pointColors = pointColors;
        cube.pointer = {};
        boundingCubes.push_back(cube);
    }
    return boundingCubes;
}

static void write_point_cloud(const char *file_name, cilantro::PointCloud3f pc)
{
    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << pc.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < pc.size(); i++)
    {
        ss << pc.points(0, i) << " " << pc.points(1, i) << " "
           << pc.points(2, i)
           << " " << int(pc.colors(0, i) * 255.f) << " "
           << int(pc.colors(1, i) * 255.f) << " "
           << int(pc.colors(2, i) * 255.f)
           << std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

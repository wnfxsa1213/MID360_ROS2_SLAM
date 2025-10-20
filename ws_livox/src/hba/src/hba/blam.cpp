#include "blam.h"
#include <Eigen/SparseCholesky>
#include <functional>
VoxelKey VoxelKey::index(double x, double y, double z, double resolution, double bias)
{
    V3D point(x, y, z);
    V3D idx = (point / resolution + V3D(bias, bias, bias)).array().floor();
    return VoxelKey(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
}
OctoTree::OctoTree(int layer, int max_layer, int min_point_num, double plane_thresh, double quater_len, const V3D &center)
    : m_layer(layer), m_max_layer(max_layer), m_min_point_num(min_point_num), m_plane_thresh(plane_thresh), m_quater_len(quater_len), m_is_valid(false), m_is_plane(false), m_center(center)
{
    m_leaves.resize(8, nullptr);
}
void OctoTree::insert(const PointType &point)
{
    m_points.push_back(point);
}
void OctoTree::buildPlanes()
{
    if (static_cast<int>(m_points.size()) < m_min_point_num)
    {
        m_is_plane = false;
        m_is_valid = false;
        return;
    }

    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.x, point.y, point.z);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    m_mean = mean;
    m_eigen_val = eigensolver.eigenvalues();
    m_eigen_vec = eigensolver.eigenvectors();
    if (m_eigen_val[0] < m_plane_thresh)
    {
        m_mean = mean;
        m_is_plane = true;
        m_is_valid = true;
        doMergePoints();
        return;
    }
    m_is_plane = false;
    m_is_valid = splitPlanes();
}
void OctoTree::doMergePoints()
{
    assert(m_is_valid && m_is_plane);
    struct AccumulatedPoint
    {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double sum_intensity = 0.0;
        double sum_lx = 0.0;
        double sum_ly = 0.0;
        double sum_lz = 0.0;
        uint32_t frame_id = 0;
        uint32_t point_id = 0;
        uint32_t count = 0;
    };

    auto make_key = [](uint32_t frame_id, uint32_t point_id) -> uint64_t {
        return (static_cast<uint64_t>(frame_id) << 32) | static_cast<uint64_t>(point_id);
    };

    std::unordered_map<uint64_t, AccumulatedPoint> id_point_map;
    id_point_map.reserve(m_points.size());
    uint32_t fallback_local_id = 0;

    for (const PointType &point : m_points)
    {
        uint32_t frame_id = point.id;
        uint32_t local_id = 0;
        if (point.time >= 0.0)
        {
            local_id = static_cast<uint32_t>(point.time);
        }
        else
        {
            local_id = fallback_local_id++;
        }

        uint64_t key = make_key(frame_id, local_id);
        AccumulatedPoint &entry = id_point_map[key];
        if (entry.count == 0)
        {
            entry.frame_id = frame_id;
            entry.point_id = local_id;
        }
        entry.sum_x += point.x;
        entry.sum_y += point.y;
        entry.sum_z += point.z;
        entry.sum_intensity += point.intensity;
        entry.sum_lx += point.lx;
        entry.sum_ly += point.ly;
        entry.sum_lz += point.lz;
        entry.count += 1;
    }

    PointVec().swap(m_merged_points);
    m_merged_points.reserve(id_point_map.size());
    for (const auto &iter : id_point_map)
    {
        const AccumulatedPoint &acc = iter.second;
        if (acc.count == 0) continue;

        PointType point;
        double denom = static_cast<double>(acc.count);
        point.x = static_cast<float>(acc.sum_x / denom);
        point.y = static_cast<float>(acc.sum_y / denom);
        point.z = static_cast<float>(acc.sum_z / denom);
        point.intensity = static_cast<float>(acc.sum_intensity / denom);
        point.lx = static_cast<float>(acc.sum_lx / denom);
        point.ly = static_cast<float>(acc.sum_ly / denom);
        point.lz = static_cast<float>(acc.sum_lz / denom);
        point.id = acc.frame_id;
        point.time = 1.0;
        m_merged_points.push_back(point);
    }
}
bool OctoTree::splitPlanes()
{
    if (m_layer >= m_max_layer - 1)
        return false;
    for (PointType &point : m_points)
    {
        int xyz[3] = {0, 0, 0};
        if (point.x > m_center.x())
            xyz[0] = 1;
        if (point.y > m_center.y())
            xyz[1] = 1;
        if (point.z > m_center.z())
            xyz[2] = 1;

        int leaf_num = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (m_leaves[leaf_num] == nullptr)
        {
            V3D shift((2 * xyz[0] - 1) * m_quater_len, (2 * xyz[1] - 1) * m_quater_len, (2 * xyz[2] - 1) * m_quater_len);
            V3D center = m_center + shift;
            m_leaves[leaf_num].reset(new OctoTree(m_layer + 1, m_max_layer, m_min_point_num, m_plane_thresh, m_quater_len / 2.0, center));
        }
        m_leaves[leaf_num]->insert(point);
    }

    PointVec().swap(m_points);
    PointVec().swap(m_merged_points);

    bool has_sub_plane = false;
    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
        {
            leaf->buildPlanes();
            if (leaf->isValid())
                has_sub_plane = true;
        }
    }
    return has_sub_plane;
}
void OctoTree::getPlanes(Vec<OctoTree *> &planes)
{
    if (m_is_plane)
    {
        planes.push_back(this);
        return;
    }
    if (!m_is_valid)
        return;

    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
            leaf->getPlanes(planes);
    }
}
int OctoTree::planeCount()
{
    if (m_is_plane)
        return 1;

    if (!m_is_valid)
        return 0;

    int count = 0;
    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
            count += leaf->planeCount();
    }
    return count;
}
double OctoTree::updateByPose(const Vec<Pose> &poses)
{
    assert(m_is_valid && m_is_plane);
    for (PointType &point : m_merged_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
    }

    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    m_mean = mean;
    m_eigen_val = eigensolver.eigenvalues();
    m_eigen_vec = eigensolver.eigenvectors();
    return m_eigen_val[0];
}
double OctoTree::evalByPose(const Vec<Pose> &poses)
{
    assert(m_is_valid && m_is_plane);
    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    return eigensolver.eigenvalues()[0];
}
M3D OctoTree::fp(const V3D &p)
{
    M3D ret = M3D::Zero();
    double denom = static_cast<double>(m_points.size());
    if (denom <= 0.0)
    {
        return ret;
    }

    ret.row(0) = (p - m_mean).transpose() *
        (m_eigen_vec.col(0) * m_eigen_vec.col(0).transpose()) / denom;

    const double eps = 1e-8;
    double gap01 = std::abs(m_eigen_val(0) - m_eigen_val(1));
    if (gap01 > eps)
    {
        ret.row(1) = (p - m_mean).transpose() *
            (m_eigen_vec.col(1) * m_eigen_vec.col(0).transpose() +
             m_eigen_vec.col(0) * m_eigen_vec.col(1).transpose()) /
            denom / gap01;
    }

    double gap02 = std::abs(m_eigen_val(0) - m_eigen_val(2));
    if (gap02 > eps)
    {
        ret.row(2) = (p - m_mean).transpose() *
            (m_eigen_vec.col(2) * m_eigen_vec.col(0).transpose() +
             m_eigen_vec.col(0) * m_eigen_vec.col(2).transpose()) /
            denom / gap02;
    }

    return ret;
}
V3D OctoTree::dp(const V3D &p)
{
    return 2.0 / static_cast<double>(m_points.size()) * ((p - m_mean).transpose() * m_eigen_vec.col(0) * m_eigen_vec.col(0).transpose()).transpose();
}
M3D OctoTree::dp2(const V3D &p1, const V3D &p2, bool equal)
{
    double n = static_cast<double>(m_points.size());
    V3D u0 = m_eigen_vec.col(0);
    if (equal)
    {
        return 2.0 / n * ((n - 1) / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * fp(p2) + m_eigen_vec * fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
    else
    {
        return 2.0 / n * (-1.0 / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * fp(p2) + m_eigen_vec * fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
}
BLAM::BLAM(const BLAMConfig &config) : m_config(config)
{
    Vec<Pose>().swap(m_poses);
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr>().swap(m_clouds);
}
void BLAM::insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose)
{
    m_poses.push_back(pose);
    m_clouds.push_back(cloud);
}

void BLAM::addPoint(const PointType &point)
{
    VoxelKey loc = VoxelKey::index(point.x, point.y, point.z, m_config.voxel_size, 0.0);
    if (m_voxel_map.find(loc) == m_voxel_map.end())
    {
        V3D center = V3D((0.5 + loc.x) * m_config.voxel_size, (0.5 + loc.y) * m_config.voxel_size, (0.5 + loc.z) * m_config.voxel_size);
        double quater_len = m_config.voxel_size / 4.0;
        m_voxel_map[loc] = std::make_shared<OctoTree>(0, m_config.max_layer, m_config.min_point_num, m_config.plane_thresh, quater_len, center);
    }
    m_voxel_map[loc]->insert(point);
}
void BLAM::buildVoxels()
{

    Vec<OctoTree *>().swap(m_planes);
    VoxelMap().swap(m_voxel_map);
    for (size_t i = 0; i < m_clouds.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_clouds[i];
        const Pose &pose = m_poses[i];
        for (size_t j = 0; j < cloud->points.size(); ++j)
        {
            const pcl::PointXYZI &point = cloud->points[j];
            PointType p;
            V3D p_vec(point.x, point.y, point.z);
            p_vec = pose.r * p_vec + pose.t;
            p.x = static_cast<float>(p_vec.x());
            p.y = static_cast<float>(p_vec.y());
            p.z = static_cast<float>(p_vec.z());
            p.lx = point.x;
            p.ly = point.y;
            p.lz = point.z;
            p.intensity = point.intensity;
            p.id = static_cast<uint32_t>(i);
            p.time = static_cast<double>(j);
            addPoint(p);
        }
    }

    for (auto &iter : m_voxel_map)
    {
        iter.second->buildPlanes();
        iter.second->getPlanes(m_planes);
    }
}
void BLAM::updateJaccAndHess()
{
    const size_t pose_dim = m_poses.size() * 6;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(pose_dim * 12);
    m_J = Eigen::VectorXd::Zero(pose_dim);
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane_ptr = m_planes[i];
        const PointVec &points = plane_ptr->mergedPoints();
        Eigen::MatrixXd D_i = Eigen::MatrixXd::Zero(3 * points.size(), 6 * points.size());
        Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(3 * points.size(), 3 * points.size());
        Eigen::MatrixXd J_i = Eigen::MatrixXd::Zero(1, 3 * points.size());
        Vec<uint32_t> idxs(points.size(), 0);
        for (size_t m = 0; m < points.size(); m++)
        {
            const PointType &point1 = points[m];
            V3D p1(point1.x, point1.y, point1.z);
            V3D pl1(point1.lx, point1.ly, point1.lz);
            idxs[m] = point1.id;
            Pose &pose = m_poses[point1.id];
            for (size_t n = 0; n < points.size(); n++)
            {
                const PointType &point2 = points[n];
                V3D p2(point2.x, point2.y, point2.z);
                H_i.block<3, 3>(m * 3, n * 3) = plane_ptr->dp2(p1, p2, m == n);
            }
            J_i.block<1, 3>(0, m * 3) = plane_ptr->dp(p1).transpose();
            D_i.block<3, 3>(m * 3, m * 3) = -pose.r * Sophus::SO3d::hat(pl1);
            D_i.block<3, 3>(m * 3, m * 3 + 3) = M3D::Identity();
        }
        Eigen::MatrixXd H_bar = D_i.transpose() * H_i * D_i; // 6 n * 6 n
        Eigen::VectorXd J_bar = (J_i * D_i).transpose();

        for (size_t m = 0; m < idxs.size(); m++)
        {
            uint32_t p_id1 = idxs[m];
            for (size_t n = m; n < idxs.size(); n++)
            {
                uint32_t p_id2 = idxs[n];
                int row_base = static_cast<int>(p_id1 * 6);
                int col_base = static_cast<int>(p_id2 * 6);
                for (int r = 0; r < 6; ++r)
                {
                    for (int c = 0; c < 6; ++c)
                    {
                        double value = H_bar(m * 6 + r, n * 6 + c);
                        if (std::abs(value) < 1e-12)
                            continue;
                        triplets.emplace_back(row_base + r, col_base + c, value);
                        if (p_id1 != p_id2)
                        {
                            triplets.emplace_back(col_base + c, row_base + r, value);
                        }
                    }
                }
            }
            m_J.segment<6>(p_id1 * 6) += J_bar.block<6, 1>(m * 6, 0);
        }
    }
    m_H.resize(pose_dim, pose_dim);
    m_H.setFromTriplets(triplets.begin(), triplets.end(), std::plus<double>());
    m_H.makeCompressed();
    m_diag_cache = m_H.diagonal();
}
void BLAM::optimize()
{
    buildVoxels();
    const size_t pose_dim = m_poses.size() * 6;
    m_J.resize(pose_dim);
    double residual = 0.0;
    bool build_hess = true;
    double u = 0.01, v = 2.0;
    Eigen::VectorXd diag;
    for (size_t i = 0; i < m_config.max_iter; i++)
    {
        if (build_hess)
        {
            residual = updatePlanesByPoses(m_poses);
            updateJaccAndHess();
            diag = m_diag_cache;
        }
        Eigen::SparseMatrix<double> Hess = m_H;
        Hess.reserve(Hess.nonZeros() + pose_dim);
        for (int idx = 0; idx < static_cast<int>(pose_dim); ++idx)
        {
            double add = u * diag(idx);
            if (add != 0.0)
            {
                Hess.coeffRef(idx, idx) += add;
            }
        }
        Hess.makeCompressed();

        double max_diag = diag.size() > 0 ? diag.maxCoeff() : 0.0;
        double min_diag = diag.size() > 0 ? diag.minCoeff() : 0.0;
        double cond_num = (min_diag > 1e-12) ? max_diag / min_diag : std::numeric_limits<double>::infinity();

        if (!std::isfinite(cond_num) || cond_num > 1e12) {
            std::cerr << "[HBA][WARN] Hessian condition number too large: " << cond_num
                      << ", skip update" << std::endl;
            u *= v;
            v *= 2;
            build_hess = false;
            continue;
        }

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(Hess);
        if (solver.info() != Eigen::Success)
        {
            u *= v;
            build_hess = false;
            continue;
        }

        Eigen::VectorXd delta = solver.solve(-m_J);
        if (solver.info() != Eigen::Success || !delta.allFinite()) {
            std::cerr << "[HBA][ERROR] Hessian solve returned non-finite delta, abort optimization"
                      << std::endl;
            break;
        }
        Vec<Pose> temp_pose(m_poses.begin(), m_poses.end());
        plusDelta(temp_pose, delta);
        double residual_new = evalPlanesByPoses(temp_pose);
        Eigen::VectorXd diag_delta = diag.cwiseProduct(delta);
        double denom = 0.5 * delta.dot(u * diag_delta - m_J);
        if (std::abs(denom) < 1e-12) {
            denom = std::copysign(1e-12, denom);
        }
        double pho = (residual - residual_new) / denom;
        if (pho > 0)
        {
            build_hess = true;
            // std::cout << "ITER : " << i << " LM ITER UPDATE" << std::endl;
            m_poses = temp_pose;
            v = 2.0;
            double q = 1 - pow(2 * pho - 1, 3);
            u *= (q < 0.33333333 ? 0.33333333 : q);
        }
        else
        {
            build_hess = false;
            u *= v;
            v *= 2;
        }
        if (std::abs(residual - residual_new) < 1e-9)
            break;
    }
}

double BLAM::updatePlanesByPoses(const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane = m_planes[i];
        residual += plane->updateByPose(poses);
    }
    return residual;
}

double BLAM::evalPlanesByPoses(const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane = m_planes[i];
        residual += plane->evalByPose(poses);
    }
    return residual;
}

int BLAM::planeCount(bool with_sub_planes)
{
    int count = 0;
    for (auto &iter : m_voxel_map)
    {
        if (with_sub_planes)
            count += iter.second->planeCount();
        else if (iter.second->isPlane())
            count += 1;
    }
    return count;
}

M6D BLAM::informationBlock(size_t row_pose, size_t col_pose) const
{
    M6D block = M6D::Zero();
    if (m_H.rows() == 0 || m_H.cols() == 0)
    {
        return block;
    }

    int row_start = static_cast<int>(row_pose * 6);
    int row_end = row_start + 6;
    int col_start = static_cast<int>(col_pose * 6);
    int col_end = col_start + 6;

    for (int col = col_start; col < col_end; ++col)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(m_H, col); it; ++it)
        {
            if (it.row() >= row_start && it.row() < row_end)
            {
                block(it.row() - row_start, col - col_start) = it.value();
            }
        }
    }
    return block;
}

void BLAM::plusDelta(Vec<Pose> &poses, const Eigen::VectorXd &x)
{
    assert(static_cast<size_t>(x.rows()) == poses.size() * 6);
    for (size_t i = 0; i < poses.size(); i++)
    {
        M3D &r = poses[i].r;
        V3D &t = poses[i].t;
        r = r * Sophus::SO3d::exp(x.segment<3>(i * 6)).matrix();
        t += x.segment<3>(i * 6 + 3);
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr BLAM::getLocalCloud()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
    Pose &first_pose = m_poses[0];

    for (OctoTree *plane : m_planes)
    {
        for (const PointType &point : plane->points())
        {
            pcl::PointXYZI p;
            Pose &cur_pose = m_poses[point.id];
            M3D r_fc = first_pose.r.transpose() * cur_pose.r;
            V3D t_fc = first_pose.r.transpose() * (cur_pose.t - first_pose.t);
            V3D p_vec(point.lx, point.ly, point.lz);
            p_vec = r_fc * p_vec + t_fc;
            p.x = p_vec[0];
            p.y = p_vec[1];
            p.z = p_vec[2];
            p.intensity = point.intensity;
            ret->push_back(p);
        }
    }
    return ret;
}

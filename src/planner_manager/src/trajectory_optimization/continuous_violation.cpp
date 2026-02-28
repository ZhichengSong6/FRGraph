# include "trajectory_optimization/continuous_violation.h"

static WorstViolation evalAtT(double t, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, 
                                const std::vector<Eigen::Vector2d>& vertices, const BezierSE2& traj){
    WorstViolation out;
    out.t = t;

    const Eigen::Vector2d p = traj.pos(t);
    const Eigen::Matrix2d R = traj.R(t);

    const int m = A.rows();
    for (int k = 0; k < m; ++k){
        const Eigen::Vector2d a = A.row(k).transpose();
        for(int i = 0; i < vertices.size(); ++i){
            const Eigen::Vector2d x = p + R * vertices[i];
            const double g = a.dot(x) - b[k];
            if (g > out.g){
                out.g = g;
                out.plane_k = k;
                out.vert_i = i;
            }
        }
    }
    return out;
}

static WorstViolation evalAtT(double t, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, 
                                const std::vector<Eigen::Vector3d>& vertices, const BezierSE3& traj){
    WorstViolation out;
    out.t = t;

    const Eigen::Vector3d p = traj.pos(t);
    const Eigen::Matrix3d R = traj.R(t);

    const int m = A.rows();
    for (int k = 0; k < m; ++k){
        const Eigen::Vector3d a = A.row(k).transpose();
        for(int i = 0; i < vertices.size(); ++i){
            const Eigen::Vector3d x = p + R * vertices[i];
            const double g = a.dot(x) - b[k];
            if (g > out.g){
                out.g = g;
                out.plane_k = k;
                out.vert_i = i;
            }
        }
    }
    return out;
}

static void bezier2dDerivativeBoundsStrict(const BezierSE2& traj, double& dp_max_norm, double& dth_max_abs){
    // derivate control points for degree-5 Bezier
    double qx_min =  std::numeric_limits<double>::infinity();
    double qx_max = -std::numeric_limits<double>::infinity();
    double qy_min =  std::numeric_limits<double>::infinity();
    double qy_max = -std::numeric_limits<double>::infinity();

    double qt_min =  std::numeric_limits<double>::infinity();
    double qt_max = -std::numeric_limits<double>::infinity();    

    for (int i = 0; i < 5; ++i){
        const Eigen::Vector2d Qi = 5.0 * (traj.P[i+1] - traj.P[i]);
        qx_min = std::min(qx_min, Qi[0]);
        qx_max = std::max(qx_max, Qi[0]);
        qy_min = std::min(qy_min, Qi[1]);
        qy_max = std::max(qy_max, Qi[1]);

        const double ti = 5.0 * (traj.Theta[i+1] - traj.Theta[i]);
        qt_min = std::min(qt_min, ti);
        qt_max = std::max(qt_max, ti);
    }

    const double dx_abs_max = std::max(std::abs(qx_min), std::abs(qx_max));
    const double dy_abs_max = std::max(std::abs(qy_min), std::abs(qy_max));
    dp_max_norm = 0.0;
    for (int i = 0; i < 5; ++i){
        const Eigen::Vector2d Qi = 5.0 * (traj.P[i+1] - traj.P[i]);
        dp_max_norm = std::max(dp_max_norm, Qi.norm());
    }

    dth_max_abs = std::max(std::abs(qt_min), std::abs(qt_max));
}

static double computeGlobalL(const Eigen::MatrixXd& A, const std::vector<Eigen::Vector2d>& vertices, 
                            const BezierSE2& traj, bool unit_normals){
    double r_max = 0.0;
    for (const auto& v : vertices){
        r_max = std::max(r_max, v.norm());
    }

    double dp_max_norm = 0.0, dth_max_abs = 0.0;
    bezier2dDerivativeBoundsStrict(traj, dp_max_norm, dth_max_abs);

    double a_max = 0.0;
    for(int k = 0; k < A.rows(); ++k){
        const double an = unit_normals ? 1.0 : A.row(k).norm();
        a_max = std::max(a_max, an);
    }
    // L = max_k ||a_k|| * ( max||p'|| + r_max * max|theta'| )
    return a_max * (dp_max_norm + r_max * dth_max_abs);
}

static void bezier3dDerivativeBoundsStrict(const BezierSE3& traj, double& dp_max_norm, double& droll_max_abs, double& dpitch_max_abs, double& dyaw_max_abs, double& omega_max){
    dp_max_norm = 0.0;
    droll_max_abs = 0.0;
    dpitch_max_abs = 0.0;
    dyaw_max_abs = 0.0;
    omega_max = 0.0;
    for (int i = 0; i < 5; ++i){
        const Eigen::Vector3d Qi = 5.0 * (traj.P[i+1] - traj.P[i]);
        dp_max_norm = std::max(dp_max_norm, Qi.norm());

        const double Qr = 5.0 * (traj.Theta[i+1][0] - traj.Theta[i][0]);
        const double Qp = 5.0 * (traj.Theta[i+1][1] - traj.Theta[i][1]);
        const double Qy = 5.0 * (traj.Theta[i+1][2] - traj.Theta[i][2]);

        droll_max_abs = std::max(droll_max_abs, std::abs(Qr));
        dpitch_max_abs = std::max(dpitch_max_abs, std::abs(Qp));
        dyaw_max_abs = std::max(dyaw_max_abs, std::abs(Qy));
    }

    const double drpy_max_norm = std::sqrt(droll_max_abs * droll_max_abs + dpitch_max_abs * dpitch_max_abs + dyaw_max_abs * dyaw_max_abs);
    omega_max = std::sqrt(3.0) * drpy_max_norm; 
}

static double computeGlobalL(const Eigen::MatrixXd& A, const std::vector<Eigen::Vector3d>& vertices, 
                            const BezierSE3& traj, bool unit_normals){
    double r_max = 0.0;
    for (const auto& v : vertices){
        r_max = std::max(r_max, v.norm());
    }
    double dp_max_norm = 0.0, droll_max_abs = 0.0, dpitch_max_abs = 0.0, dyaw_max_abs = 0.0, omega_max = 0.0;
    bezier3dDerivativeBoundsStrict(traj, dp_max_norm, droll_max_abs, dpitch_max_abs, dyaw_max_abs, omega_max);

    double a_max = 0.0;
    for(int k = 0; k < A.rows(); ++k){
        const double an = unit_normals ? 1.0 : A.row(k).norm();
        a_max = std::max(a_max, an);
    }
    // L = a_max * ( max||p'|| + r_max * omega_max )
    return a_max * (dp_max_norm + r_max * omega_max);
}

static IntervalNode makeNode(
    double tL, double tR,
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector2d>& verts,
    const BezierSE2& traj,
    double L)
{
  IntervalNode node;
  node.tL = tL;
  node.tR = tR;

  const double tc = 0.5 * (tL + tR);
  node.mid = evalAtT(tc, A, b, verts, traj);

  const double dt = (tR - tL);
  node.U = node.mid.g + L * dt * 0.5;  // Lipschitz certificate bound
  return node;
}

static IntervalNode makeNode(
    double tL, double tR,
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector3d>& verts,
    const BezierSE3& traj,
    double L)
{
  IntervalNode node;
  node.tL = tL;
  node.tR = tR;

  const double tc = 0.5 * (tL + tR);
  node.mid = evalAtT(tc, A, b, verts, traj);

  const double dt = (tR - tL);
  node.U = node.mid.g + L * dt * 0.5;  // Lipschitz certificate bound
  return node;
}

WorstViolation FindWorstViolationContinuous2D(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector2d>& robot_vertices,
    const BezierSE2& traj,
    const VerifyOptions& opt)
{
    WorstViolation worst;
    worst.g = -1e100;

    const double L = computeGlobalL(A, robot_vertices, traj, opt.unit_normals);

    std::priority_queue<IntervalNode, std::vector<IntervalNode>, NodeCmp> pq;
    pq.push(makeNode(0.0, 1.0, A, b, robot_vertices, traj, L));   

    int expanded = 0;

    while (!pq.empty() && expanded < opt.max_nodes) {
        IntervalNode cur = pq.top();
        pq.pop();
        ++expanded;
        if (cur.mid.g > worst.g) {
            worst = cur.mid;
        }

        if (cur.U <= opt.eps){
            continue; // safe interval
        }

        const double dt = cur.tR - cur.tL;
        if (dt <= opt.min_dt){
            // cannot subdivide further
            worst.safe = (worst.g <= opt.eps);
            return worst;
        }

        // subdivide
        const double tm = 0.5 * (cur.tL + cur.tR);
        pq.push(makeNode(cur.tL, tm, A, b, robot_vertices, traj, L));
        pq.push(makeNode(tm, cur.tR, A, b, robot_vertices, traj, L));
    }

    // if pq is empty all intervals are safe
    if(pq.empty() && worst.g <= opt.eps){
        worst.safe = true;
        return worst;
    }

    // otherwise return worst found so far
    worst.safe = false;
    return worst;
}

WorstViolation FindWorstViolationContinuous3D(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector3d>& robot_vertices,
    const BezierSE3& traj,
    const VerifyOptions& opt)
{
    WorstViolation worst;
    worst.g = -1e100;

    const double L = computeGlobalL(A, robot_vertices, traj, opt.unit_normals);

    std::priority_queue<IntervalNode, std::vector<IntervalNode>, NodeCmp> pq;
    pq.push(makeNode(0.0, 1.0, A, b, robot_vertices, traj, L));

    int expanded = 0;

    while (!pq.empty() && expanded < opt.max_nodes) {
        IntervalNode cur = pq.top();
        pq.pop();
        ++expanded;
        if (cur.mid.g > worst.g) {
            worst = cur.mid;
        }

        if (cur.U <= opt.eps){
            continue; // safe interval
        }

        const double dt = cur.tR - cur.tL;
        if (dt <= opt.min_dt){
            // cannot subdivide further
            worst.safe = (worst.g <= opt.eps);
            return worst;
        }

        // subdivide
        const double tm = 0.5 * (cur.tL + cur.tR);
        pq.push(makeNode(cur.tL, tm, A, b, robot_vertices, traj, L));
        pq.push(makeNode(tm, cur.tR, A, b, robot_vertices, traj, L));
    }

    // if pq is empty all intervals are safe
    if(pq.empty() && worst.g <= opt.eps){
        worst.safe = true;
        return worst;
    }

    // otherwise return worst found so far
    worst.safe = false;
    return worst;
}

static inline Eigen::Matrix2d S2D() {
    Eigen::Matrix2d S;
    S << 0.0, -1.0,
        1.0,  0.0;
    return S;
}

std::vector<ViolatedPlaneAtT> collectViolatedPlanesAtT(
    double t_star,
    const Eigen::MatrixXd& A,        // m x 2
    const Eigen::VectorXd& b,        // m
    const std::vector<Eigen::Vector2d>& verts_body,
    const BezierSE2& traj,
    double eps_add,
    int topK)
{
    std::vector<ViolatedPlaneAtT> violated_planes;
    const Eigen::Vector2d p = traj.pos(t_star);
    const Eigen::Matrix2d R = traj.R(t_star);
    const Eigen::Matrix2d S = S2D();

    const int m = A.rows();
    violated_planes.reserve(m);

    for (int k = 0; k < m; ++k){
        Eigen::Vector2d a = A.row(k).transpose();
        double bk = b[k];

        int best_i = -1;
        double best_val = -std::numeric_limits<double>::infinity();
        for(int i = 0; i < (int)verts_body.size(); ++i){
            // double val = a.dot(p + R * verts_body[i]);
            double val = a.dot(R * verts_body[i]);
            if (val > best_val){
                best_val = val;
                best_i = i;
            }
        }

        const Eigen::Vector2d x = p + R * verts_body[best_i];
        const double g = a.dot(x) - bk;
        if (g > eps_add) {
            ViolatedPlaneAtT vp;
            vp.k = k;
            vp.i = best_i;
            vp.g = g;
            vp.a = a;
            vp.b = bk;
            vp.dgdtheta = a.dot(R * (S * verts_body[best_i])); // a^T R S v
            violated_planes.push_back(vp);
        }
    }

    if (topK > 0 && (int)violated_planes.size() > topK) {
        std::nth_element(violated_planes.begin(), violated_planes.begin()+topK, violated_planes.end(),
        [](const ViolatedPlaneAtT& x, const ViolatedPlaneAtT& y){ return x.g > y.g; });
        violated_planes.resize(topK);
    }

    // sort descending by g
    std::sort(violated_planes.begin(), violated_planes.end(),
        [](const ViolatedPlaneAtT& x, const ViolatedPlaneAtT& y){ return x.g > y.g; });

    return violated_planes;
}

std::vector<ViolatedPlaneAtT3D> collectViolatedPlanesAtT(
    double t_star,
    const Eigen::MatrixXd& A,        // m x 2
    const Eigen::VectorXd& b,        // m
    const std::vector<Eigen::Vector3d>& verts_body,
    const BezierSE3& traj,
    double eps_add,
    int topK)
{
    std::vector<ViolatedPlaneAtT3D> violated_planes;
    const Eigen::Vector3d p = traj.pos(t_star);
    const Eigen::Matrix3d R = traj.R(t_star);

    const int m = A.rows();
    violated_planes.reserve(m);

    for (int k = 0; k < m; ++k){
        Eigen::Vector3d a = A.row(k).transpose();
        double bk = b[k];

        int best_i = -1;
        double best_val = -std::numeric_limits<double>::infinity();
        for(int i = 0; i < (int)verts_body.size(); ++i){
            double val = a.dot(p + R * verts_body[i]);
            if (val > best_val){
                best_val = val;
                best_i = i;
            }
        }

        const Eigen::Vector3d x = p + R * verts_body[best_i];
        const double g = a.dot(x) - bk;
        if (g > eps_add) {
            ViolatedPlaneAtT3D vp;
            vp.k = k;
            vp.i = best_i;
            vp.g = g;
            vp.a = a;
            vp.b = bk;
            // ---- attitude gradient wrt small body rotation delta_phi (body frame) ----
            // d g ≈ (v × (R^T a))^T delta_phi   
            const Eigen::Vector3d v = verts_body[best_i];
            const Eigen::Vector3d a_body = R.transpose() * a;
            vp.dgdphi_body = v.cross(a_body);

            violated_planes.push_back(vp);  
        }       
    }
    if (topK > 0 && (int)violated_planes.size() > topK) {
        std::nth_element(violated_planes.begin(), violated_planes.begin()+topK, violated_planes.end(),
        [](const ViolatedPlaneAtT3D& x, const ViolatedPlaneAtT3D& y){ return x.g > y.g; });
        violated_planes.resize(topK);
    }
    // sort descending by g
    std::sort(violated_planes.begin(), violated_planes.end(),
        [](const ViolatedPlaneAtT3D& x, const ViolatedPlaneAtT3D& y){ return x.g > y.g; });

    return violated_planes;
}

// choose Top-K among internal control points {1,2,3,4} by Bernstein weight B[j]
std::vector<int> selectTopKControlPoints(double t_star, int Kcp) {
    auto B = BezierSE2::bernstein5(t_star);
    std::vector<int> idx = {1,2,3,4};
    std::sort(idx.begin(), idx.end(), [&](int a, int b){ return B[a] > B[b]; });
    if (Kcp < (int)idx.size()) idx.resize(Kcp);
    return idx;
}

// Apply dx (solution) to traj for selected control points
static void applyDeltaToBezier(BezierSE2& traj,
                               const std::vector<int>& ctrl_idx,
                               const Eigen::VectorXd& x,
                               double alpha)
{
    const int Kcp = (int)ctrl_idx.size();
    // variable layout: for each cp: [dPx, dPy, dTheta], then slack at end
    for (int cpi = 0; cpi < Kcp; ++cpi) {
        const int j = ctrl_idx[cpi];
        traj.P[j].x() += alpha * x(3*cpi + 0);
        traj.P[j].y() += alpha * x(3*cpi + 1);
        traj.Theta[j] += alpha * x(3*cpi + 2);
    }
}

static void applyDeltaToBezier(BezierSE3& traj,
                               const std::vector<int>& ctrl_idx,
                               const Eigen::VectorXd& x,
                               double alpha)
{
    const int Kcp = (int)ctrl_idx.size();
    // variable layout: for each cp: [dPx, dPy, dPz, dRoll, dPitch, dYaw], then slack at end
    for (int cpi = 0; cpi < Kcp; ++cpi) {
        const int j = ctrl_idx[cpi];
        traj.P[j].x() += alpha * x(6*cpi + 0);
        traj.P[j].y() += alpha * x(6*cpi + 1);
        traj.P[j].z() += alpha * x(6*cpi + 2);
        traj.Theta[j][0] += alpha * x(6*cpi + 3); // roll
        traj.Theta[j][1] += alpha * x(6*cpi + 4); // pitch
        traj.Theta[j][2] += alpha * x(6*cpi + 5); // yaw
    }
}

static bool solveRepairQP_PIQP(
    double t_star,
    const std::vector<int>& ctrl_idx,                  // selected CP indices in {1..4}
    const std::vector<ViolatedPlaneAtT>& violated,     // usually Top-K planes by g
    double margin,
    double delta_p,
    double delta_th,
    double w_p,
    double w_th,
    double w_slack,
    Eigen::VectorXd& x_sol)
{
    const double inf = std::numeric_limits<double>::infinity();
    const int Kcp = (int)ctrl_idx.size();
    const int n = 3 * Kcp + 1; // decision variables: [dPx, dPy, dTheta for each CP] + slack
    const int m = (int)violated.size();

    // Build QP matrices
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(n);

    for (int cpi = 0; cpi < Kcp; ++cpi){
        P(3*cpi + 0, 3*cpi + 0) = w_p;   // weight for dPx^2
        P(3*cpi + 1, 3*cpi + 1) = w_p;   // weight for dPy^2
        P(3*cpi + 2, 3*cpi + 2) = w_th;  // weight for dTheta^2
    }
    P(n-1, n-1) = w_slack; // weight for slack variable

    // tiny regularization to avoid singular P if you set some weights 0
    for (int i = 0; i < n; ++i) P(i,i) += 1e-9;

    // ---- Build inequality Gx <= h_u (and h_l = -inf) ----
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(m, n);
    Eigen::VectorXd h_l = Eigen::VectorXd::Constant(m, -inf);
    Eigen::VectorXd h_u = Eigen::VectorXd::Zero(m);

    auto B = BezierSE2::bernstein5(t_star);

    for (int r = 0; r < m; ++r){
        const auto& vp = violated[r];
        for (int cpi = 0; cpi < Kcp; ++cpi){
            const int j = ctrl_idx[cpi];
            const double bj = B[j];
            G(r, 3*cpi + 0) = bj * vp.a.x();
            G(r, 3*cpi + 1) = bj * vp.a.y();
            G(r, 3*cpi + 2) = bj * vp.dgdtheta;
        }
        // slack with -1
        G(r, n-1) = -1.0;
        // RHS: -g - margin
        h_u(r) = -vp.g - margin;
    }
    // Box constraints x_l <= x <= x_u (trust region + slack >= 0)
    Eigen::VectorXd x_l = Eigen::VectorXd::Constant(n, -inf);
    Eigen::VectorXd x_u = Eigen::VectorXd::Constant(n, inf);
    for (int cpi = 0; cpi < Kcp; ++cpi){
        x_l(3*cpi + 0) = -delta_p; // dPx >= -delta_p
        x_u(3*cpi + 0) = delta_p;  // dPx <= delta_p
        x_l(3*cpi + 1) = -delta_p; // dPy >= -delta_p
        x_u(3*cpi + 1) = delta_p;  // dPy <= delta_p
        x_l(3*cpi + 2) = -delta_th; // dTheta >= -delta_th
        x_u(3*cpi + 2) = delta_th;  // dTheta <= delta_th
    }
    x_l(n-1) = 0.0; // slack >= 0

        // ---- Solve with PIQP ----
    piqp::DenseSolver<double> solver;
    solver.settings().verbose = false;
    solver.settings().compute_timings = false;

    // no equality constraints => pass piqp::nullopt for A,b
    solver.setup(P, c, piqp::nullopt, piqp::nullopt, G, h_l, h_u, x_l, x_u);
    piqp::Status status = solver.solve();

    if (status != piqp::Status::PIQP_SOLVED) {
        // ROS_WARN("PIQP failed, status=%d", (int)status);
        return false;
    }

    x_sol = solver.result().x;
    return true;
}

bool RepairOnce_PIQP(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector2d>& robot_vertices,
    BezierSE2& traj,
    const VerifyOptions& opt,
    int Kcp,
    int topKplanes,
    double eps_add,
    double margin,
    double delta_p,
    double delta_th,
    double w_p,
    double w_th,
    double w_slack)
{
    WorstViolation w0 = FindWorstViolationContinuous2D(A, b, robot_vertices, traj, opt);
    if (w0.safe) {
        return true; // already safe
    }

    // collect violated planes at t*
    std::vector<ViolatedPlaneAtT> violated = collectViolatedPlanesAtT(w0.t, A, b, robot_vertices, traj, eps_add, topKplanes);
    if (violated.empty()) {
        std::cout << "[RepairOnce_PIQP] Warning: no violated planes found at t*=" << w0.t << " even though w0 is violated. This should not happen." << std::endl;
        return true; // no violated planes found (should not happen since w0 is violated)
    }

    // select Top-K control points near t*
    std::vector<int> ctrl_idx = selectTopKControlPoints(w0.t, Kcp);

    // solve QP to get delta
    Eigen::VectorXd x_sol;
    if (!solveRepairQP_PIQP(w0.t, ctrl_idx, violated, margin, delta_p, delta_th, w_p, w_th, w_slack, x_sol)){
        return false; // QP failed
    }

    // line search to apply delta
    double alpha = 1.0;
    for (int ls = 0; ls < 6; ++ls){
        BezierSE2 traj_candidate = traj; 
        applyDeltaToBezier(traj_candidate, ctrl_idx, x_sol, alpha);
        WorstViolation w_candidate = FindWorstViolationContinuous2D(A, b, robot_vertices, traj_candidate, opt);
        if (w_candidate.g < w0.g){
            traj = traj_candidate; // accept
            return true;
        }
        alpha *= 0.5; // reduce step size
    }
    return false; // failed to find improvement
}

bool RepairOnce_PIQP(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector3d>& robot_vertices,
    BezierSE3& traj,
    const VerifyOptions& opt,
    int Kcp,
    int topKplanes,
    double eps_add,
    double margin,
    double delta_p,
    double delta_th,
    double w_p,
    double w_th,
    double w_slack)
{
    // WorstViolation w0 = FindWorstViolationContinuous3D(A, b, robot_vertices, traj, opt);
    // if (w0.safe) {
    //     return true; // already safe
    // }

    // // collect violated planes at t*
    // std::vector<ViolatedPlaneAtT> violated = collectViolatedPlanesAtT(w0.t, A, b, robot_vertices, traj, eps_add, topKplanes);
    // if (violated.empty()) {
    //     std::cout << "[RepairOnce_PIQP] Warning: no violated planes found at t*=" << w0.t << " even though w0 is violated. This should not happen." << std::endl;
    //     return true; // no violated planes found (should not happen since w0 is violated)
    // }

    // // select Top-K control points near t*
    // std::vector<int> ctrl_idx = selectTopKControlPoints(w0.t, Kcp);

    // // solve QP to get delta
    // Eigen::VectorXd x_sol;
    // if (!solveRepairQP_PIQP(w0.t, ctrl_idx, violated, margin, delta_p, delta_th, w_p, w_th, w_slack, x_sol)){
    //     return false; // QP failed
    // }

    // // line search to apply delta
    // double alpha = 1.0;
    // for (int ls = 0; ls < 6; ++ls){
    //     BezierSE3 traj_candidate = traj; 
    //     applyDeltaToBezier(traj_candidate, ctrl_idx, x_sol, alpha);
    //     WorstViolation w_candidate = FindWorstViolationContinuous3D(A, b, robot_vertices, traj_candidate, opt);
    //     if (w_candidate.g < w0.g){
    //         traj = traj_candidate; // accept
    //         return true;
    //     }
    //     alpha *= 0.5; // reduce step size
    // }
    // return false; // failed to find improvement
}
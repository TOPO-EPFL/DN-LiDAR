#include <iostream>
#include <iomanip>

#include "ROAMestimation/ROAMestimation.h"
#include "ROAMimu/IMUIntegralHandler.h"

#include "stamped_csv.hpp"
#include "initial_guess_generator.hpp"

namespace RE = ROAMestimation;
namespace RI = ROAMimu;

int main(int argc, char *argv[])
{

    /* ------------------------------------------------------------------------------------------
       --------------------- CONFIGURATION PARAMETERS -------------------------------------------
       ------------------------------------------------------------------------------------------ */

    // here all configuration parameters specific for the current platform and sensors are defined

    // ######### general configuration

    double t0 = 396400.0;
    double tend = 397000.0;

    Eigen::VectorXd gravity(1);
    gravity << 9.80665;

    // ######### GNSS configuration

    // tansformation bringing vectors from GNSS frame to body frame
    Eigen::VectorXd T_body_GNSS(7);
    T_body_GNSS << 0.292, -0.233, -1.524, // GNSS^b (lever-arm) in meters
        1.0, 0.0, 0.0, 0.0;               // q^b_GNSS (boresight quaternion) // N.B. not important for GNSS

    // GNSS measurement file
    // format:
    // timestamp, X [m], Y [m], Z [m], X std [m], Y std [m], Z std [m]
    std::string GNSS_meas_filename = "../data/GNSS_lat0=46.566641300900_lon0=6.512265144377_short.txt";

    // ######### IMU configuration

    double imu_sampling_period = 0.002; // 500 Hz
    int integrate_n_meas = 5;           // use IMU pre-integration so that the effective pose rate is 100 Hz

    // tansformation bringing vectors from IMU frame to body frame
    Eigen::VectorXd T_body_IMU(7);
    T_body_IMU << -0.0493, 0.22185, -0.1658,                                             // IMU^b (lever-arm) in meters
        0.9999441086054481, 0.0004179023817722, 0.0015752428857903, -0.0104462257636836; // q^b_IMU (boresight quaternion)

    // parameters defining the stochastic model for the IMU
    double gyro_GM_beta = 0.0138719562;     // 1/tau, where tau is the Gauss-Markov correlation time in seconds
    double gyro_GM_var = 2.520344e-12;      // variance of the Gauss-Markov innovation, in rad^2/s^2 (per sample, at IMU rate)
    double gyro_GM_sampling_period = 1.0;   // estimate samples of the GM process every (GM process subsampling)
    double gyro_WN_variance = 1.377041e-06; // variance of the gyroscope white noise, in rad^2/s^2 (discrete time units, per sample) 

    double acc_GM_beta = 0.0224348399;      // 1/tau, where tau is the Gauss-Markov correlation time in seconds TODO
    double acc_GM_var = 3.084702e-11;       // variance of the Gauss-Markov innovation, in m^2/s^4 (per sample, at IMU rate)
    double acc_GM_sampling_period = 1.0;    // estimate samples of the GM process every
    double acc_WN_variance = 0.0001275208;  // variance of the gyroscope white noise, in m^2/s^4 (discrete time units, per sample)

    // Accelerometer measurement file
    // format:
    // timestamp, X [m/s^2], Y [m/s^2], Z [m/s^2]
    std::string acc_meas_filename = "../data/accelerometer_short.txt";

    // Gyroscope measurement file
    // format:
    // timestamp, X [rad/s], Y [rad/s], Z [rad/s]
    std::string gyro_meas_filename = "../data/gyroscope_short.txt";

    // ######### LiDAR configuration

    // tansformation bringing vectors from LiDAR frame to body frame
    Eigen::VectorXd T_body_LiDAR(7);
    T_body_LiDAR << -0.042,  0.183, -0.021,                                              // LiDAR^b (lever-arm) in meters
        0.500523884813, 0.500338779229, -0.497800992431, 0.501329351490;                 // q^b_LiDAR (boresight quaternion)

    double LiDAR_p2p_variance = 0.008; // point-to-point correspondences residual variance,  as in Equation 2   

    // LiDAR measurement file
    // format: (as in Equation 3)
    // timestamp 1, timestamp 2, v1^L x [m], v1^L y [m], v1^L z [m], v2^L x [m], v2^L y [m], v2^L z [m]
    // records have to be sorted such that timestamp 2 < timestamp 1
    std::string LiDAR_meas_filename = "../data/correspondences.txt";

    /* ------------------------------------------------------------------------------------------
       --------------------- SOLVER CONFIGURATION -----------------------------------------------
       ------------------------------------------------------------------------------------------ */

    std::cerr << " * Configuring solver ... " << std::endl;

    // instatiate the DN solver (FactorGraphFilter object)
    RE::FactorGraphFilter *f = RE::FactorGraphFilterFactory::getNewFactorGraphFilter();

    f->setSolverMethod(RE::GaussNewton);
    f->setChi2Threshold(1e2); // stop if overall chi2 does not improve by more than 1e-3

    int ret = system("mkdir /tmp/roamfree");
    f->setLowLevelLogging(true, "/tmp/roamfree");

    f->addConstantParameter(RE::Euclidean1D, "gravity", gravity, true);

    /* ------------------------------------------------------------------------------------------
       --------------------- SENSOR CONFIGURATION -----------------------------------------------
       ------------------------------------------------------------------------------------------ */

    // ######### GNSS configuration

    f->addSensor("GNSS", RE::AbsolutePosition, false, false); // non master sensor, non sequential sensor
    f->setSensorFrame("GNSS", T_body_GNSS);

    StampedCsv GNSS_data(GNSS_meas_filename, 6);

    // a delay parameter necessary to account for the GNSS measurements possibly not being synchronous with IMU ones.
    // N.B. ROAMFREE supports to some extent the estimation of sensor synchronization delays
    // this parameter is not used in this example (fixed = true, value = 0) but it is necessary for configuration
    RE::ParameterWrapper_Ptr GNSS_delay_par = f->addConstantParameter("GNSS_SE3IntDelay", 0.0, true);

    // This is the variance of pseudo observations used in SE3 interpolation (0 edges in Figure 4)    
    // This value should be as small as possible but large enough to avoid numerical problems
    // N.B. 0 edges are not used for GNSS in this example because it happens to be synchronous with the IMU in the data available
    Eigen::MatrixXd GNSS_SE3interp_pseudoObsCov = 1e-9 * Eigen::MatrixXd::Identity(6, 6);

    // ######### Gyroscope stochastic processes

    // gyroscope random constant
    Eigen::VectorXd gyro_RC_x0(3); // initial value
    gyro_RC_x0 << 0.0, 0.0, 0.0;

    RE::ParameterWrapper_Ptr gyro_RC_par = f->addConstantParameter( // adds a parameter constant in time
        RE::Euclidean3D,
        "IMUintegralDeltaP_Bw_RC",
        gyro_RC_x0,
        false);

    // gyroscope first-order Gauss-Markov

    Eigen::VectorXd gyro_GM_x0(3); // initial value
    gyro_GM_x0 << 0.0, 0.0, 0.0;

    RE::ParameterWrapper_Ptr gyro_GM_par = f->addLinearlyInterpolatedParameter( // adds a parameter that varies in time
        RE::Euclidean3D,
        "IMUintegralDeltaP_Bw_GM",
        gyro_GM_x0,
        false,
        gyro_GM_sampling_period);

    // we need to convert the variance of the Gauss-Markov innovations
    // to continuous time units (per Herz)
    double gyro_GM_var_cnt = gyro_GM_var * (2.0 * gyro_GM_beta) / (1.0 - exp(-2.0 * gyro_GM_beta * imu_sampling_period));

    gyro_GM_par->setProcessModelType(RE::GaussMarkov);
    gyro_GM_par->setGaussMarkovNoiseCov(gyro_GM_var_cnt * Eigen::MatrixXd::Identity(3, 3));
    gyro_GM_par->setGaussMarkovBeta(gyro_GM_beta * Eigen::VectorXd::Ones(3));

    // create the actual bias parameter, the sum of all stochastic processes defined so far

    RE::ParameterWrapper_Ptr gyro_bias = f->addParameterBlender(
        RE::Euclidean3D,
        "IMUintegralDeltaP_Bw",
        RE::ParameterWrapperVector_Ptr(
            new RE::ParameterWrapperVector{
                gyro_RC_par,
                gyro_GM_par}));

    
    // ######### Accelerometer stochastic processes

    // accelerometer random constant
    Eigen::VectorXd acc_RC_x0(3); // initial value
    acc_RC_x0 << 0.0, 0.0, 0.0;

    RE::ParameterWrapper_Ptr acc_RC_par = f->addConstantParameter( // adds a parameter constant in time
        RE::Euclidean3D,
        "IMUintegralDeltaP_Ba_RC",
        acc_RC_x0,
        false);

    // accelerometer first-order Gauss-Markov

    Eigen::VectorXd acc_GM_x0(3); // initial value
    acc_GM_x0 << 0.0, 0.0, 0.0;

    RE::ParameterWrapper_Ptr acc_GM_par = f->addLinearlyInterpolatedParameter( // adds a parameter that varies in time
        RE::Euclidean3D,
        "IMUintegralDeltaP_Ba_GM",
        acc_GM_x0,
        false,
        acc_GM_sampling_period);

    // we need to convert the variance of the Gauss-Markov innovations
    // to continuous time units (per Herz)
    double acc_GM_var_cnt = acc_GM_var * (2.0 * acc_GM_beta) / (1.0 - exp(-2.0 * acc_GM_beta * imu_sampling_period));

    acc_GM_par->setProcessModelType(RE::GaussMarkov);
    acc_GM_par->setGaussMarkovNoiseCov(acc_GM_var_cnt * Eigen::MatrixXd::Identity(3, 3));
    acc_GM_par->setGaussMarkovBeta(acc_GM_beta * Eigen::VectorXd::Ones(3));

    // create the actual bias parameter, the sum of all stochastic processes defined so far

    RE::ParameterWrapper_Ptr acc_bias = f->addParameterBlender(
        RE::Euclidean3D,
        "IMUintegralDeltaP_Ba",
        RE::ParameterWrapperVector_Ptr(
            new RE::ParameterWrapperVector{
                acc_RC_par,
                acc_GM_par}));

    // ######### IMU configuration

    // Object that support IMU pre-integration

    RI::IMUIntegralHandler hndl(
        f,
        "IMUintegral",
        integrate_n_meas,
        imu_sampling_period,
        acc_bias,
        gyro_bias,
        T_body_IMU);

    // fill the matrix that contains the IMU white noise variances (6x6 matrix, first accelerometer, next gyroscope)

    Eigen::Matrix<double, 6, 6> &IMU_WN = hndl.getSensorNoises();
    IMU_WN.setZero();
    IMU_WN.diagonal().head(3) << acc_WN_variance * Eigen::Vector3d::Ones();
    IMU_WN.diagonal().tail(3) << gyro_WN_variance * Eigen::Vector3d::Ones();

    StampedCsv acc_data(acc_meas_filename, 3);
    StampedCsv gyro_data(gyro_meas_filename, 3);

    // ######### LiDAR configuration    

    f->addSensor("LiDAR", RE::LiDARTieFeatures, false, false); // non master sensor, non sequential sensor
    f->setSensorFrame("LiDAR", T_body_LiDAR);

    StampedCsv LiDAR_data(LiDAR_meas_filename, 7);

    // a delay parameter necessary to account for the LiDAR measurements possibly not being synchronous with IMU ones.
    // this parameter is not used in this example (fixed = true, value = 0) but it is necessary for configuration
    RE::ParameterWrapper_Ptr LiDAR_delay_par = f->addConstantParameter("LiDAR_SE3IntDelay", 0.0, true);

    // This is the variance of pseudo observations used in SE3 interpolation (0 edges in Figure 4)
    // This value should be as small as possible but large enough to avoid numerical problems
    Eigen::MatrixXd LiDAR_SE3interp_pseudoObsCov = 1e-9 * Eigen::MatrixXd::Identity(6, 6);

    /* ------------------------------------------------------------------------------------------
       --------------------- INITIAL GUESS ------------------------------------------------------
       ------------------------------------------------------------------------------------------ */

    // we determine the initialization for the DN solver applying a Savitzky–Golay filter to 
    // the provided GNSS positions to obtain an approximation of the body frame position at
    // the frequency of the IMU. The orientation is derived assuming that the body frame
    // mounting is either Front-Left-Up or Front-Right-Down and the x axis is tangent to 
    // the body frame trajectory.

    InitialGuessGenerator ig_gen(FRONTRIGHTDOWN, GNSS_meas_filename);

    Eigen::VectorXd last_ig(7);
    last_ig << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    /* ------------------------------------------------------------------------------------------
       --------------------- MAIN LOOP ----------------------------------------------------------
       ---------- where all measurement are added to the dn ------------------------------------- */


    std::cerr << " * Adding measurements ... " << std::endl;    

    // initialize the DN (done by the IMUIntegralHandler since we are using IMU pre-integration)    

    Eigen::VectorXd initial_pose(7);
    initial_pose = ig_gen.initialGuessAt(t0, last_ig); 
    last_ig = initial_pose;

    hndl.init(true, t0, initial_pose);

    // add the assumption (prior) that the GM process at t0 is equal to 0 +- innovation variance 
    
    f->addPriorOnTimeVaryingParameter(
        RE::Euclidean3DPrior,
        "IMUintegralDeltaP_Ba_GM",
        t0,
        acc_GM_x0,
        acc_GM_var * Eigen::MatrixXd::Identity(3,3)
    );

    f->addPriorOnTimeVaryingParameter(
        RE::Euclidean3DPrior,
        "IMUintegralDeltaP_Bw_GM",
        t0,
        gyro_GM_x0,
        gyro_GM_var * Eigen::MatrixXd::Identity(3,3)
    );

    // read one GNSS measurement
    bool GNSS_valid = GNSS_data.next();
    assert(GNSS_valid);    

    // read one LiDAR measurement 
    bool LiDAR_valid = LiDAR_data.next();
    assert(LiDAR_valid);

    while (true) {
        // get the next measurements for accelerometer and gyroscope

        bool acc_valid = acc_data.next();
        bool gyro_valid = gyro_data.next();

        assert(acc_valid && gyro_valid);
        assert(acc_data.getTimestamp() == gyro_data.getTimestamp());

        double t_imu = acc_data.getTimestamp();

        if (t_imu < t0 - 1e-6) { 
            continue; // skip measurements before t0
        }
        if (t_imu > tend + 1e-6) { 
            break; // we are done adding measurements
        }

        // do the IMU pre-integration
        // will return true every integrate_n_meas steps        
        bool integration_completed = hndl.step(
                t_imu, 
                acc_data.getCurrent().data(),
                gyro_data.getCurrent().data()
        );

        if (integration_completed == true) {
            // if here, a new pose has been added to the DN graph
            // we need to initialize it using the InitialGuessGenerator
            RE::PoseVertexWrapper_Ptr newest_pose = f->getNewestPose();

            assert(newest_pose->getTimestamp() == t_imu);

            newest_pose->setEstimate(
                ig_gen.initialGuessAt(t_imu, last_ig)
            );
            last_ig = newest_pose->getEstimate();

            // now add the GNSS measurements        

            while (GNSS_data.isValid()) {
                
                if (GNSS_data.getTimestamp() < t0) {
                    GNSS_data.next(); // skip measurements before t0
                    continue; 
                }

                if (GNSS_data.getTimestamp() > t_imu + 1e-7) {
                    break; // for those after t_imu, wait that more IMU readings are considered
                }

                // With the data used for this example this method always returns one existing 
                // pose (without creating new ones) because GNSS timestamps corrsepond to certain IMU ones 
                // the 0 edges mechanism it is used here for the sake of generality

                RE::PoseVertexWrapper_Ptr GNSS_pose = f->addInterpolatingPose(
                    GNSS_data.getTimestamp(), 
                    GNSS_delay_par, 
                    GNSS_SE3interp_pseudoObsCov
                );

                assert(GNSS_pose);

                // create the covariance matrix of this GNSS observation
                Eigen::MatrixXd GNSS_covariance(3,3);
                GNSS_covariance.setZero();
                GNSS_covariance.diagonal() = GNSS_data.getCurrent().tail(3).array().square(); // in the file we have standard deviations

                RE::MeasurementEdgeWrapper_Ptr GNSS_edge = f->addMeasurement(
                    "GNSS", 
                    GNSS_data.getTimestamp(),
                    GNSS_data.getCurrent().head(3), 
                    GNSS_covariance, 
                    GNSS_pose
                );

                assert(GNSS_edge);

                GNSS_data.next();
            }

            // now add the LiDAR measurements

            while (LiDAR_data.isValid()) {
                double t1 = LiDAR_data.getTimestamp();
                double t2 = LiDAR_data.getCurrent()(0); // the first column of the measurement record is the second timestamp

                if (t1 < t0 || t2 < t0) {
                    LiDAR_data.next(); // skip measurements in which either t1 or t2 are before t0
                    continue; 
                }

                if (t1 > t_imu + 1e-7) {
                    break; // for those after t_imu, wait that more IMU readings are considered
                }

                // add the two new poses and related 0 Edges as in Figure 4

                RE::PoseVertexWrapper_Ptr p2 = f->addInterpolatingPose(
                    t2, LiDAR_delay_par, LiDAR_SE3interp_pseudoObsCov
                ); // oldest pose
                RE::PoseVertexWrapper_Ptr p1 = f->addInterpolatingPose(
                    t1, LiDAR_delay_par, LiDAR_SE3interp_pseudoObsCov
                ); // newest pose

                assert(p1 != NULL);
                assert(p2 != NULL);

                RE::MeasurementEdgeWrapper_Ptr LiDAR_edge = f->addMeasurement(
                    "LiDAR", 
                    t1, 
                    LiDAR_data.getCurrent().tail(6), // last six elements are [ v1^L v2^L ] 
                    LiDAR_p2p_variance * Eigen::MatrixXd::Identity(3,3), 
                    p1, // newest pose
                    p2 // oldest pose
                );

                assert(LiDAR_edge);

                LiDAR_data.next();
            }
        }
    }

    std::cerr << " * Solving ... ";

    f->estimate(10);

    std::cout << " done, final chi2 = " << std::fixed << std::setprecision(3) << f->getChi2() << std::endl;

    std::cerr << " * Compressing results in ../data/trajectories/T2-DNC.tar.gz" << std::endl;

    system("bash -c \"cd /tmp/roamfree; tar -zcf '/tmp/T2-DNC.tar.gz' * \"");
    system("mv /tmp/T2-DNC.tar.gz ../data/trajectories");

    std::cerr << " * Removing temporaries" << std::endl;

    system("rm -rf /tmp/roamfree");

}
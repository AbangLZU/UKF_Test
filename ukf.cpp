#include <iostream>
#include "ukf.h"

UKF::UKF() {
    //TODO Auto-generated constructor stub
    Init();
}

UKF::~UKF() {
    //TODO Auto-generated destructor stub
}

void UKF::Init() {

}

/*******************************************************************************
* Programming assignment functions: 
*******************************************************************************/


void UKF::GenerateSigmaPoints(MatrixXd *Xsig_out) {

    //set state dimension
    int n_x = 5;

    //define spreading parameter
    double lambda = 3 - n_x;

    //set example state
    VectorXd x = VectorXd(n_x);
    x << 5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;

    //set example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
            -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
            0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
            -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
            -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

    //calculate square root of P
    MatrixXd A = P.llt().matrixL();

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //your code goes here

    //calculate sigma points ...
    //set sigma points as columns of matrix Xsig

    Xsig.col(0) = x;
    // 2 - nx+1 columns
    for (int i = 0; i < n_x; i++) {
        Xsig.col(i + 1) = x + (sqrt(lambda + n_x) * A.col(i));
        Xsig.col(i + 1 + n_x) = x - (sqrt(lambda + n_x) * A.col(i));
    }


/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
//    std::cout << "Xsig = " << std::endl << Xsig << std::endl;

    //write result
    *Xsig_out = Xsig;

/* expected result:
   Xsig =
    5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
      1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
    2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
    0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
    0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
*/

}

void UKF::AugmentedSigmaPoints(MatrixXd *Xsig_out) {

    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a = 0.2;

    //Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd = 0.2;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //set example state
    VectorXd x = VectorXd(n_x);
    x << 5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;

    //create example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
            -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
            0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
            -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
            -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //create augmented mean state
    //create augmented covariance matrix
    //create square root matrix
    //create augmented sigma points
    x_aug.head(5) = x;

    Xsig_aug.col(0) = x_aug;
    MatrixXd Q = MatrixXd(2, 2);
    Q << std_a * std_a, 0,
            0, std_yawdd * std_yawdd;

    P_aug.topLeftCorner(5, 5) = P;
    P_aug.bottomRightCorner(2, 2) = Q;
    MatrixXd A = P_aug.llt().matrixL();

    for (int i = 0; i < n_aug; i++) {
        Xsig_aug.col(i + 1) = x_aug + (sqrt(lambda + n_aug) * A.col(i));
        Xsig_aug.col(i + 1 + n_aug) = x_aug - (sqrt(lambda + n_aug) * A.col(i));
    }


/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

    //write result
    *Xsig_out = Xsig_aug;

/* expected result:
   Xsig_aug =
  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
    1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
       0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
       0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
*/

}

/*******************************************************************************
* Programming assignment functions:
*******************************************************************************/

void UKF::SigmaPointPrediction(MatrixXd *Xsig_out) {

    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //create example sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
    Xsig_aug <<
             5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
            1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
            2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
            0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
            0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
            0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
            0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;


    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

    double delta_t = 0.1; //time diff in sec
/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //predict sigma points
    //avoid division by zero
    //write predicted sigma points into right column

    for(int i =0; i < (2 * n_aug + 1); i++){
        VectorXd input_x = Xsig_aug.col(i);
        float px = input_x[0];
        float py = input_x[1];
        float v = input_x[2];
        float psi = input_x[3];
        float psi_dot = input_x[4];
        float mu_a = input_x[5];
        float mu_psi_dot_dot = input_x[6];

        VectorXd term2 = VectorXd(5);
        VectorXd term3 = VectorXd(5);

        VectorXd result = VectorXd(5);
        if(psi_dot < 0.001){
            term2 << v * cos(psi) * delta_t, v * sin(psi) * delta_t, 0, psi_dot * delta_t, 0;
            term3 << 0.5 * delta_t*delta_t * cos(psi) * mu_a,
                    0.5 * delta_t*delta_t * sin(psi) * mu_a,
                    delta_t * mu_a,
                    0.5 * delta_t*delta_t * mu_psi_dot_dot,
                    delta_t * mu_psi_dot_dot;
            result = Xsig_aug.col(i).head(5) + term2 + term3;
        } else{
            term2 << (v/psi_dot) * (sin(psi + psi_dot * delta_t) - sin(psi)),
                    (v/psi_dot) * (-cos(psi + psi_dot * delta_t) + cos(psi)),
                    0,
                    psi_dot * delta_t,
                    0;

            term3 << 0.5 * delta_t*delta_t * cos(psi) * mu_a,
                    0.5 * delta_t*delta_t * sin(psi) * mu_a,
                    delta_t * mu_a,
                    0.5 * delta_t*delta_t * mu_psi_dot_dot,
                    delta_t * mu_psi_dot_dot;
            result = Xsig_aug.col(i).head(5) + term2 + term3;
        }

        Xsig_pred.col(i) = result;
    }


/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

    //write result
    *Xsig_out = Xsig_pred;

}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //create example matrix with predicted sigma points
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_pred <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);

    //create vector for predicted state
    VectorXd x = VectorXd(n_x);

    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x, n_x);


/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //set weights
    //predict state mean
    //predict state covariance matrix

    weights[0] = lambda/ (lambda + n_aug);
    for(int i=1; i < (2*n_aug+1); i++){
        weights[i] = 1/(2 * (lambda + n_aug));
    }
    x.fill(0.0);
    for(int i=0; i<2*n_aug+1; i++){
        x = x+ weights(i) * Xsig_pred.col(i);
    }

    P.fill(0.0);
    for(int i=0;  i<2*n_aug+1; i++){
        VectorXd x_diff = Xsig_pred.col(i) - x;
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        P = P + weights[i] * x_diff * x_diff.transpose();
    }




/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Predicted state" << std::endl;
    std::cout << x << std::endl;
    std::cout << "Predicted covariance matrix" << std::endl;
    std::cout << P << std::endl;

    //write result
    *x_out = x;
    *P_out = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //set vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);
    double weight_0 = lambda/(lambda+n_aug);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug+1; i++) {
        double weight = 0.5/(n_aug+lambda);
        weights(i) = weight;
    }

    //radar measurement noise standard deviation radius in m
    double std_radr = 0.3;

    //radar measurement noise standard deviation angle in rad
    double std_radphi = 0.0175;

    //radar measurement noise standard deviation radius change in m/s
    double std_radrd = 0.1;

    //create example matrix with predicted sigma points
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_pred <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //transform sigma points into measurement space
    //calculate mean predicted measurement
    //calculate measurement covariance matrix S


/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "z_pred: " << std::endl << z_pred << std::endl;
    std::cout << "S: " << std::endl << S << std::endl;

    //write result
    *z_out = z_pred;
    *S_out = S;
}
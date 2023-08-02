//#include <iostream>
//#include <fstream>
//#include <cmath>
//#include <vector>
//
//using namespace std;
//
//// Function prototypes
//void hypodromefn(double s, double radius, double line_length, double& P_x_derivative, double& P_y_derivative, double& P_x, double& P_y);
//double velocity(double W, double tBlend, double tInstant, double tFinal);
//
//int main() {
//    vector<double> sspace(100);
//    vector<double> tspace(100);
//    vector<double> Px(100);
//    vector<double> Py(100);
//    vector<double> Pdx(100);
//    vector<double> Pdy(100);
//    vector<double> sDot(100);
//
//    double tFinal = 10.0;
//
//    // Populate sspace and tspace vectors
//    for (int i = 0; i < 100; i++) {
//        sspace[i] = i * 3.0 / 99.0;
//        tspace[i] = i * tFinal / 99.0;
//    }
//
//    // Calculate positions and velocities
//    for (int i = 0; i < 100; i++) {
//        double s = sspace[i];
//        double t = tspace[i];
//        hypodromefn(s, 3.0, 5.0, Pdx[i], Pdy[i], Px[i], Py[i]);
//        sDot[i] = velocity(5.0, 3.0, t, tFinal);
//    }
//
//    // Save the calculated values to a CSV file
//    ofstream outputFile("calculated_variables.csv");
//    if (outputFile.is_open()) {
//        outputFile << "s, t, Px, Py, Pdx, Pdy, sDot" << endl;
//        for (int i = 0; i < 100; i++) {
//            outputFile << sspace[i] << ", " << tspace[i] << ", " << Px[i] << ", " << Py[i] << ", " << Pdx[i] << ", " << Pdy[i] << ", " << sDot[i] << endl;
//        }
//        outputFile.close();
//        cout << "File saved successfully." << endl;
//    }
//    else {
//        cout << "Unable to open the file." << endl;
//    }
//
//    return 0;
//}
//
//void hypodromefn(double s, double radius, double line_length, double& P_x_derivative, double& P_y_derivative, double& P_x, double& P_y) {
//    double M_PI = 3.1415;
//    double alpha = (2 * line_length) / (radius * M_PI) + 1;
//
//    if (s <= 1) {
//        P_x = -radius * cos(s * M_PI / 2);
//        P_y = -radius * sin(s * M_PI / 2);
//
//        P_x_derivative = radius * (M_PI / 2) * sin(s * M_PI / 2);
//        P_y_derivative = -radius * (M_PI / 2) * cos(s * M_PI / 2);
//    }
//    else if (s <= alpha && s > 1) {
//        P_x = (s - 1) / (alpha - 1) * line_length;
//        P_y = -radius;
//
//        P_x_derivative = radius * M_PI / 2;
//        P_y_derivative = 0;
//    }
//    else {
//        P_x = radius * sin((s - alpha) * M_PI / 2) + line_length;
//        P_y = -radius * cos((s - alpha) * M_PI / 2);
//
//        P_x_derivative = radius * M_PI / 2 * cos((s - alpha) * M_PI / 2);
//        P_y_derivative = radius * M_PI / 2 * sin((s - alpha) * M_PI / 2);
//    }
//}
//
//double velocity(double W, double tBlend, double tInstant, double tFinal) {
//    double sDot;
//
//    if (tInstant <= tBlend)
//        sDot = W * tInstant;
//    else if (tInstant > tBlend && tInstant < tFinal - tBlend)
//        sDot = W * tBlend;
//    else
//        sDot = W * (tFinal - tInstant);
//
//    return sDot;
//}
//
//

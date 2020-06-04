/**************************************************************************
 * Library to calculate polynomial regression in Arduino.
 * 
 * Simple, efficient and memory efficient method to 
 * calculate mean values, linear regressions or 
 * parabolas from pairs of values.
 * 
 * Based on Cramers rule  for fitting of a polynomial 
 * regression model using least square method.
 * For a very good explanation, please see:
 * https://neutrium.net/mathematics/least-squares-fitting-of-a-polynomial/
 * 
 * and thanks to Cubiwan:
 * https://github.com/cubiwan/LinearRegressino
 * and Chandu yadav:
 * https://www.tutorialspoint.com/cplusplus-program-to-compute-determinant-of-a-matrix
 * for the inspiration on how to implement it
 * and Martin R.
 * https://codereview.stackexchange.com/questions/204135/determinant-using-gauss-elimination
 * for the tip about partial pivoting...
 * 
 * 
 * Hague Nusseck @ electricidea
 * v1.4 24.May.2020
 * https://github.com/electricidea/xxxx
 * 
 * Changelog:
 * v1.2 = - first version ready with "Laplace expansion" Method to calculate determinants
 * v1.3 = - Gaussian triangular method for the determination of determinants (faster!)
 *        - added name and tag
 *        - added max_x and min_x
 *        - established aij Matrix notation
 *        - String output for formular
 * v1.4 = - dynamic matrix sizes according to initialized order
 *        - functions to estimate max and min y values over the known range of x
 *        - add function count() to return N
 * 
 * 
 * Distributed as-is; no warranty is given.
 * 
 **************************************************************************
 * 
 * ==== How to use it: ====
 * 
 * 1.) intitialize the fitting object:
 * 
 *          curve_fit fit_1 = curve_fit(1);
 * NOTE:
 * degree = 0 --> A constant (or: The Average of y values)
 * degree = 1 --> A first order polynomial (or: linear regression)
 * degree = 2 --> A second order polynomial (or: parabolic curve)
 * degree = 3 --> A third order polynomial
 * Note: 
 * Generally the degree is not limited, but from the 6th or 7th 
 * degree the floating-point arithmetic reaches its limits and 
 * there may be inaccuracies.
 * 
 * In principle, there is no limit for the degree of polynomials, 
 * but from the 6th or 7th degree on, first calculation errors appear
 * due to the inaccurate floating-point arithmetic. 
 * 
 * 2.) Let the function solve the polynomial regression model 
 *     by adding pairs of x and y values:
 * 
 *          fit_1.learn(0,5);
 *          fit_1.learn(1,18.65);
 *              ...
 *          fit_1.learn(10,52.4);
 * 
 * During each of these steps the polynomial representation is calculated. 
 * The value pairs are not stored. As a result, the computing time is 
 * higher, but the class can be used more flexible.
 * 
 * 3.) print out the calculated coefficients:
 * 
 *          double coeff_values[2];
 *          fit_1.get_coefficients(coeff_values);
 *          Serial.printf("y= a1*x + a0\n\na1= %.2f\na0= %.2f", 
 *                        coeff_values[1], coeff_values[0]);
 * 
 * 4.) Calculate the y values for a new x value:
 * 
 *          new_y = fit_1.predict(new_x);
 * 
 * Some NOTES:
 * At any time additional pairs of values can be added to improve 
 * the calculation / allow further learning.....
 * The value pairs are not stored. They cannot be read out again 
 * or changed afterwards.
 * 
 * 
 * ==== How the Math works: ==== 
 * 
 * Representation of a kth order polynomial
 * y = ak*x^k +⋯+ a1*x + a0
 * 
 * with
 * a	:	Polynomial coefficient
 * k	:	The degree of the polynomial
 * N	:	The number of points to be regressed
 *  
 * The coefficients of the polynomial regression model (ak,ak−1,⋯,a1) may be 
 * determined by solving the following system of linear equations:
 * 
 * |                                             |   |    |   |              |
 * |    N          SUM(xi)     ...    SUM(xi^k)  |   | a0 |   |    SUM(yi)   |
 * |  SUM(xi)     SUM(xi^2 )   ...   SUM(xi^k+1) |   | a1 |   |  SUM(xi*yi)  |
 * |    ...         ...        ...       ...     | * | .. | = |     ...      |
 * | SUM(xi^k)  SUM(xi^k+1)    ...    SUM(xi^2k) |   | ak |   | SUM(xi^k*yi) |
 * |                                             |   |    |   |              |
 * 
 * Cramer's rule allows you to solve the linear system of equations to find the 
 * regression coefficients using the determinants of the square matrix M. Each 
 * of the coefficients ak may be determined using the following equation:
 * 
 * ak = det(Mi) / det(M)
 * 
 * Where Mi is the matrix M with the ith column replaced with the column 
 * vector b (remembering the system is presented in the form Ma=b). 
 * For example M0 could be calculated as follows:
 * 
 * 
 *      |                                                |  
 *      |     SUM(yi)     SUM(xi)      ...    SUM(xi^k)  | 
 *      |   SUM(xi*yi)   SUM(xi^2 )    ...   SUM(xi^k+1) | 
 * M0 = |      ...          ...        ...       ...     | 
 *      | SUM(xi^k*yi)   SUM(xi^k+1)   ...    SUM(xi^2k) |  
 *      |                                                |  
 * 
 * Example for an linear regression (polynominal 1th order)
 * 
 *  y = a1*x + a0
 * 
 * |                     |   |    |   |            |
 * |   N       SUM(xi)   |   | a0 |   |   SUM(yi)  |
 * | SUM(xi)  SUM(xi^2)  | * | a1 | = | SUM(xi*yi) |
 * |                     |   |    |   |            | 
 * 
 * 
 *      |                     | 
 *      |   N       SUM(xi)   | 
 *  M = | SUM(xi)  SUM(xi^2)  | 
 *      |                     |  
 * 
 *      |                        | 
 *      |   SUM(yi)    SUM(xi)   | 
 * M0 = | SUM(xi*yi)  SUM(xi^2)  | 
 *      |                        |  
 * 
 *      |                      | 
 *      |   N       SUM(yi)    | 
 * M1 = | SUM(xi)  SUM(xi*yi)  | 
 *      |                      |  
 * 
 * a0 = det(M0) / det(M)
 * a1 = det(M1) / det(M)
 * 
 * How to calculate the determinants of a 2x2 matrix M:
 * 
 *     |         |
 *     | a11 a12 |
 * det | a21 a22 | = a11*a22 – a12*a21
 *     |         |  
 * 
 ***************************************************************************/

#include "Arduino.h"
#include "curve_fit.h"

//==============================================================
// the constructor
curve_fit::curve_fit(uint8_t degree) {
    tag = 0;
    name = "";
    curve_fit::init(degree);
}

//==============================================================
// Initialization of the fit object
// degree = 0 --> A constant (or: The Average of y values)
// degree = 1 --> A first order polynomial (or: linear regression)
// degree = 2 --> A second order polynomial (or: parabolic curve)
// degree = 3 --> A third order polynomial
// Note: 
// Generally the degree is not limited, but from the 6th or 7th 
// degree the floating-point arithmetic reaches its limits and 
// there may be inaccuracies.
// this function can be used to change the degree at runtime
bool curve_fit::init(uint8_t degree) {
    order = degree;
    free(a);
    free(b);
    free(M);
    // the matrix dimension need to order + 1 for the calculations
    a = (double*) malloc((order+1)*sizeof(double));
    b = (double*) malloc((order+1)*sizeof(double));
    M = (double*) malloc((order+1)*(order+1)*sizeof(double));
    // if malloc fails, set order to -1
    // and return false
    if(!a || !b || !M) {
		order = -1;
        N = 0;
        return false;
	}
    curve_fit::reset();
    return true;
}

//==============================================================
// clear all calculated coefficients and buffered values 
// set all values back to 0.0 to start a new calculation
void curve_fit::reset() {
    // initialize the matrices with 0.0
    for(int i = 0; i <= order; i++) {
        for(int j = 0; j <= order; j++)
            M[Mindex(i,j)] = 0.0;
        a[i] = 0.0;
        b[i] = 0.0;
    }
    // reset the number of used x, y pairs
    N = 0;
    max_x_ = 0.0;
    min_x_ = 0.0;
}

//==============================================================
// calculate the determinant by following Gaussian Method:
// Use elementary row operations to bring the matrix into 
// a row echelon form, to be able to calculated the determinant 
// by using the diagonal values.
double curve_fit::determinant( double *mainmatrix) {   
    double det = 1;
    // order < 2 = matrix dimention up to 2x2
    // for these matrix sizes, the determinants can be calculate directly
    if (order < 2) {
        if(order == -1)
            // A matrix without dimension?
            // would say the deteminat is zero....
            det = 0;
        if(order == 0)
            // The determinant of a 1x1 matrix is the element of the matrix itself
            det = mainmatrix[0];
        if(order == 1)
            // a 2x2 matrix can be calculated directly:
            //       |         |                       |            |
            //       | a11 a12 |                       | M[0]  M[1] |
            //   det | a21 a22 | = a11*a22 – a12*a21 = | M[2]  M[3] | = M[0]*M[3] - M[1]*M[2]
            //       |         |                       |            | 
            det = ((mainmatrix[0] * mainmatrix[3]) - (mainmatrix[1] * mainmatrix[2]));
    } else {   
        // otherwise, elementary row operations are required to bring the 
        // matrix into a row echelon form.
        // First, create a work-matrix with the same dimensions
        // and fill it with the data from the main matrix
        double workmatrix[(order+1)*(order+1)];
        for(int i = 0; i < order+1; ++i)
            for(int j = 0; j < order+1; ++j)
                workmatrix[Mindex(i,j)] = mainmatrix[Mindex(i,j)];
        // here are some hints about Gaussian Elimination with Partial Pivoting:
        // https://youtu.be/euIXYdyjlqo
        // https://www.youtube.com/watch?v=SQ98-eImrgA
        // Now, go throw each column
        for (int i = 0; i < order+1; ++i) {
            // find the largest element for partial pivoting
            double pivotElement = workmatrix[Mindex(i,i)];
            int pivotRow = i;
            for (int row = i + 1; row < order+1; ++row) {
                if (fabs(workmatrix[Mindex(row,i)]) > fabs(pivotElement)) {
                    pivotElement = workmatrix[Mindex(row,i)];
                    pivotRow = row;
                }
            }
            // important:
            // there is no solution, if the pivot element is zero!
            if (pivotElement == 0.0) {
                return 0.0;
            }
            // if the found pivot element is not in the actual row,
            // we need to swap the rows..
            if (pivotRow != i) {
                // swapping
                for (int k = 0; k < order+1; ++k) { 
                    double t = workmatrix[Mindex(i,k)]; 
                    workmatrix[Mindex(i,k)] = workmatrix[Mindex(pivotRow,k)];
                    workmatrix[Mindex(pivotRow,k)] = t;
                }
                // change the sign of the determinant after swapping a line!
                det *= -1.0;
            }
            // The determinant is calculated by a multiplication of the diagonal
            // values of the upper triangle matrix part.
            // in this case, the pivot element of each column
            det *= pivotElement;
            // Gaussian triangular steps to reach a row echelon form of the matrix
            for (int row = i + 1; row < order+1; ++row) {
                for (int col = i + 1; col < order+1; ++col) {
                    workmatrix[Mindex(row,col)] -= workmatrix[Mindex(row,i)] * workmatrix[Mindex(i,col)] / pivotElement;
                }
            }
        }

    }
    return det;
}

//==============================================================
// learn:
// Adding a new pair of x and y values and
// solving the polynomial regression model with Cramers rule
void curve_fit::learn(double x, double y) {
    ++N;
    // find min and max values for x
    if(N == 1) {
        max_x_ = x;
        min_x_ = x;
    } else {
        if(x < min_x_)
            min_x_ = x;
        if(x > max_x_)
            max_x_ = x;
    }
    // order is -1 when the matrices are not allocated (undefines)
    if(order > -1) {
        // Example for matrix values
        // A second-order polynomial (or: parabolic curve)
        //      |                                 |  
        //      |     N       SUM(xi)   SUM(xi^2) |    M[0] M[1] M[2] 
        //  M = |  SUM(xi)   SUM(xi^2)  SUM(xi^3) |  = M[3] M[4] M[5] 
        //      | SUM(xi^2)  SUM(xi^3)  SUM(xi^4) |    M[6] M[7] M[8] 
        //      |                                 |  
        // Matrix notation:
        // aij for each element of the matrix:
        //       |                         |   |             |   |                |
        //       | a_i1_j1 a_i1_j2 a_i1_j3 |   | a11 a12 a13 |   | M[0] M[1] M[2] |
        //   M = | a_i2_j1 a_i2_j2 a_i2_j3 | = | a21 a22 a23 | = | M[3] M[4] M[5] |
        //       | a_i3_j1 a_i3_j2 a_i3_j3 |   | a31 a32 a33 |   | M[6] M[7] M[8] |
        //       |                         |   |             |   |                |  
        // to calculate the matrix index based on i and j, the function Mindex(i, j)
        // can be used..
        // 
        // First, add the new x values to the existing values of the matrix M[]:
        for(int j = 0; j < order+1; j++) {
            for(int i = 0; i < order+1; i++) {
                // fill the first column with pow() for each element
                if(j==0) {
                    M[Mindex(i, j)] += pow(x, i);
                } else {
                    if(i < order){
                        // fill with already calculated values from 
                        // the previous column to save calulation power
                        M[Mindex(i, j)] = M[Mindex(i+1, j-1)];
                    } else {
                        // calculate only last element of the column with pow()
                        M[Mindex(i, j)] += pow(x, i+j);
                    }
                }
            }
        }
        // fill the first element with the number of samples
        M[0] = N;

        // Secondly add the new y value to the vector b[]
        //      |              |
        //      |    SUM(yi)   |   b[0]
        //  b = |  SUM(xi*yi)  | = b[1]
        //      | SUM(xi^2*yi) |   b[2]
        //      |              | 
        for(int n = 0; n < order+1; n++) {
            b[n] += (pow(x, n)*y);
        }

        // Thirdly calculate the coefficients by dividing the determinants
        //  a0=det(M0)/det(M)
        //  a1=det(M1)/det(M)
        //  ...
        //  an=det(Mn)/det(M)
    
        double det_M = determinant(M);
        // create a work-matrix to calculate M0, M1, ... , Mn
        // according to Cramer's rule
        double Mn[(order+1)*(order+1)];
        for(int n = 0; n < order+1; n++) {
            for(int j = 0; j < order+1; j++) {
                for(int i = 0; i < order+1; i++) {
                    if(j == n)
                        Mn[Mindex(i, j)] = b[i];
                    else
                        Mn[Mindex(i, j)] = M[Mindex(i, j)];
                }
            }
            // calculate the coefficients by dividing the determinants
            double det_Mn = determinant(Mn);
            a[n] = det_Mn / det_M;
        }
    }
}

//==============================================================
// predict:
// returning the predicted y values of a given x values
// based on the calculated (learned) polynomial regression model
double curve_fit::predict(double x) {
    double y = 0.0;
    for(int i = 0; i <= order; i++)
        y = y + pow(x, i) * a[i];
    return y;
}

//==============================================================
// predict:
// returning the predicted y values of a given x values
// based on the calculated (learned) polynomial regression model
// if x is outside the learned range, y is replaced with outside_value
double curve_fit::predict(double x, double outside_value) {
    double y =0.0;
    if(x >  max_x_ || x < min_x_){
        y = outside_value;
    } else {
        y = predict(x);
    }
    return y;
}

//==============================================================
// Fills the given field with the current coefficients. 
// the values field need to have the right size!
void curve_fit::get_coefficients(double values[]) {
    // order 0 -->  y = a[0]
    // order 1 -->  y = a[1]*x + a[0]
    // order 2 -->  y = a[2]*x^2 + a[1]*x + a[0]
    for(int i = 0; i <= order; ++i)
        values[i] = a[i];
}

//==============================================================
// Return the formula as String with all coefficients. 
// The number of decimal places can be set the optional parameter.
String curve_fit::get_formula(uint8_t decimals) {
    // order n -->  y = a[n]*x^n + ... + a[1]*x + a[0]
    String formula = "("+String(N)+") y= ";
    for(int i = order; i >= 0; --i) {
        // at a '+' if the value is positiv
        if(a[i] > 0 && i < order)
            formula += "+";
        if(i > 1) {
            formula += String(a[i],decimals)+"x^"+i+" ";
        } else {
            if(i== 1)
                formula += String(a[i],decimals)+"x ";
            else
                formula += String(a[i],decimals);
        }
    }
    return formula;
}

//==============================================================
// return the actual order. 
uint32_t curve_fit::get_order() {
   return order; 
}

//==============================================================
// return the matrix index based on i and j
// M[0][0] = M[0]
// M[0][3] = M[3]
// M[i][j] = M[(i*(order+1))+j]
uint32_t curve_fit::Mindex(uint32_t i, uint32_t j) {
    return (i*(order+1))+j;
}

//==============================================================
// return an estimation of the max y value over the existing x 
// range (min_x .. max_x)
// The optional parameter "steps" can be used to define the
// number of steps between min_x and max_x (default = 100)
double curve_fit::estimate_max_y(uint32_t steps) {
    if(steps == 0)
        steps = 1;
    double stepwidth = (max_x_ - min_x_) / steps;
    double max_y_ = predict(min_x_);
    for(int i = 1; i <= steps; ++i) {
        if(predict(min_x_ + (i*stepwidth)) > max_y_)
            max_y_ = predict(min_x_ + (i*stepwidth));
    }
    return max_y_;
}

//==============================================================
// return an estimation of the min y value over the existing x 
// range (min_x .. max_x)
// The optional parameter "steps" can be used to define the
// number of steps between min_x and max_x (default = 100)
double curve_fit::estimate_min_y(uint32_t steps) {
    if(steps == 0)
        steps = 1;
    double stepwidth = (max_x_ - min_x_) / steps;
    double min_y_ = predict(min_x_);
    for(int i = 1; i <= steps; ++i) {
        if(predict(min_x_ + (i*stepwidth)) < min_y_)
            min_y_ = predict(min_x_ + (i*stepwidth));
    }
    return min_y_;
}

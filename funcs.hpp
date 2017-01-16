#ifndef FUNCS_HPP
#define FUNCS_HPP

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdarg.h>
#include <time.h>

/*
extern Mat mu;
extern Mat P;
extern Mat mu_xbiw_norm;
extern Mat yib0;
extern Mat xbiwHat;
extern Mat xbiiwHat;
extern Mat yib00;
extern Mat xwbiHat;
extern vector<Mat> Rw2biHat;
extern Mat xwbiiHat;
extern vector<Mat> Rw2biiHat;
*/
/// TODO
#define ATD(i,j) at<double>(i,j)

/* For the functions below, see funcs.cpp for the definition and further description */

/* start a timer to measure time */
void tic();

/* stop the timer, also return elapsed time in seconds */
double toc();

/* "matrix is of floating point type" */
bool mfp(const cv::Mat mat);

/* "find not equal to zero" */
cv::Mat find_neqz(const cv::Mat colvector);

/* Return true if all elements are less than or equal to zero */
bool all_leqz(const cv::Mat src);

/* finds indices in a matrix that are 9000 or over */
cv::Mat its_over_9000(const cv::Mat mat);

/* show whether elements in matrix are contained in a set */
cv::Mat is_member(const cv::Mat mat, const cv::Mat set);

/* show whether elements in matrix are NOT contained in a set */
cv::Mat is_not_member(const cv::Mat mat, const cv::Mat set);

/* create a larger matrix by combining a series of matrices horizontally (copy to a destination matrix) */
void horiz_concat(int numMats, cv::Mat & dest, ...);

/* create a larger matrix by combining a series of matrices horizontally (return the matrix as output) */
cv::Mat horiz_concat(int numMats, ...);

/* create a larger matrix by combining a series of matrices vertically (copy to a destination matrix) */
void vert_concat(int numMats, cv::Mat & dest, ...);

/* create a larger matrix by combining a series of matrices vertically (return the matrix as output) */
cv::Mat vert_concat(int numMats, ...);

/* create a larger matrix by combining a series of matrices (copy to a destination matrix) */
void mat_concat(int rows, int cols, cv::Mat & dest, ...);

/* create a larger matrix by combining a series of matrices (return the matrix as output) */
cv::Mat mat_concat(int rows, int cols, ...);

/* check whether two matrices are equivalent */
int check_equal_int(const cv::Mat mat1, const cv::Mat mat2);

/* assign a matrix to have the same size, type, and values as the source matrix */
void assign_mat(cv::Mat & dest, const cv::Mat src);

/* (Same as above, but overloaded for matrix expressions) */
void assign_mat(cv::Mat & dest, const cv::MatExpr src);

/* assigns values in destination matrix to match particular values (defined by ranges) in the source matrix */
/* NOTE: This overloaded version is not generally used in the main code directly. See definition. */
void assign_mat(cv::Mat & dest, const cv::Mat src, cv::Range dRowRange, cv::Range dColRange, cv::Range sRowRange, cv::Range sColRange);

/* Same as above, but this overloaded version is more "convenient" */
void assign_mat(cv::Mat & dest, const cv::Mat src, int dRowSt, int dRowEn, int dColSt, int dColEn, int sRowSt, int sRowEn, int sColSt, int sColEn);

/* assign values in a matrix's row(s) to the values in another matrix's row(s) */
void assign_row(cv::Mat & dest, const cv::Mat src, int dRowSt, int dRowEn, int sRowSt, int sRowEn);

/* assign values in a matrix's column(s) to the values in another matrix's column(s) */
void assign_col(cv::Mat & dest, const cv::Mat src, int dColSt, int dColEn, int sColSt, int sColEn);

/* assigns a value of type double to a single element in a matrix */
void assign_elem_double(cv::Mat & dest, double k, int row, int col);

/* assigns a value of type int to a single element in a matrix */
void assign_elem_int(cv::Mat & dest, int k, int row, int col);

/* finds the maximum value in a matrix of floating point type */
double max_double(const cv::Mat src);

/* finds the maximum value in a matrix of floating point type */
double max_double(const cv::Mat src, int & idxOfMax);

/* finds the maximum value in a matrix of integer type */
int max_int(const cv::Mat src);

/* finds the mean of a matrix of floating point type */
double mean_double(const cv::Mat src);

/* creates a range of values, similar to a:b in matlab, where a and b are integers */
/* NOTE: although the params are ints, the Mat will be double */
cv::Mat range_double(int start, int end);

/* creates a range of values, similar to a:b in matlab, where a and b are integers */
cv::Mat range_int(int start, int end);

/* similar to m(r, c), where m is a matrix, and r and c are column vectors of indices */
cv::Mat mat_from_indices(const cv::Mat src, const cv::Mat r, const cv::Mat c);

/* given a list of rows to "choose" in r, create a matrix from these rows from src matrix */
cv::Mat select_rows(const cv::Mat src, const cv::Mat r);

/* given a list of columns to "choose" in c, create a matrix from these columns from src matrix */
cv::Mat select_cols(const cv::Mat src, const cv::Mat c);

cv::Mat vector_to_Mat_double(const std::vector<double> v);

std::vector<double> Mat_to_vector_double(const cv::Mat mat);

void write_mat_to_file(std::ofstream & f, cv::Mat mat);

// Assuming file is csv
cv::Mat read_mat_from_file(const std::string filename);

cv::Mat linspace(double a, double b, int n);

template <class T>
std::string to_str(T t)
{
	std::ostringstream convert;
	convert << t;
	return convert.str();
};

#endif

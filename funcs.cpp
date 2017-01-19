#include "funcs.hpp"
#include "opencv2/core.hpp"

using namespace cv;

/* the "start" time for when tic() is called */
double tic_time = 0;

/*
	start a timer to measure time 

	INPUTS:	None
	RETURNS: None
	OUTPUTS: None 
*/
void tic()
{
	tic_time = clock();
}

/*
	stop the timer, also return elapsed time in seconds 

	INPUTS:	None
	RETURNS: elapsed time in seconds from when tic() was called
	OUTPUTS: None 
*/
double toc()
{
	/* take the difference between stop and start time, divide to convert to seconds */
	return (clock() - tic_time)/1000000.0;
}

/*
	"matrix is of floating point type"

	INPUTS: mat - matrix to check for floating pointedness
	RETURNS: true if matrix is floating point type, else false
	OUTPUTS: None 
*/
bool mfp(const Mat mat)
{
	if (mat.type() == CV_64F || mat.type() == CV_32F)
		return true;
	else
		return false;
}

/* 
	"find not equal to zero" 
	find the indices of elements that are not equal to zero
	
	INPUTS: colvector - a matrix containing one column
	RETURNS: the indices of the matrix which do not contain zero
	OUTPUTS: None 
*/
Mat find_neqz(const Mat colvector)
{	
	int cnt = 0;
	Mat indices;
	if (mfp(colvector))
		for (int i = 0; i < colvector.rows; i++)
		{
			if (colvector.at<double>(i, 0) != 0)
			{
				/* append to col vector of indices */
				assign_elem_int(indices, i, cnt, 0);
				cnt++;
			}
		}
	else
		for (int i = 0; i < colvector.rows; i++)
		{
			if (colvector.at<int>(i, 0) != 0)
			{
				/* append to col vector of indices */
				assign_elem_int(indices, i, cnt, 0);
				cnt++;
			}
		}

	return indices;
}

/*
	Return true if all elements are less than or equal to zero
	
	INPUTS: src - the matrix to check values of and stuff
	RETURNS: true if all less or equal to zero
	OUTPUTS: NOTHIN'
*/
bool all_leqz(const Mat src)
{	
	if (mfp(src))
		for (int i = 0; i < src.total(); i++)
		{
			if (src.at<double>(i) > 0)
			{
				/* Nope.jpg */
				return false;
			}
		}
	else
		for (int i = 0; i < src.total(); i++)
		{
			if (src.at<int>(i) > 0)
			{
				/* Nope.jpg */
				return false;
			}
		}

	return true;
}

/* 
	finds indices in a matrix that are 9000 or over
	
	INPUTS: mat - the input matrix whose values we'll check
	RETURNS: the indices of the matrix which are 9000 or over
	OUTPUTS: None 

	See also:
		https://www.youtube.com/watch?v=SiMHTK15Pik
*/
Mat its_over_9000(const Mat mat)
{
	CV_Assert(mat.cols == 1);

	Mat indices;
	int c = 0; // count
	for (int i = 0; i < mat.rows; i++)
	{
		if (mat.at<double>(i) >= 9000)
		{
			assign_elem_int(indices, i, 0, c);
			c++;
		}
	}
	return indices;
}

/*
	show whether elements in matrix are contained in a set 

	INPUTS: mat - the input matrix whose values we'll check
			src - the set of values to check for
	RETURNS: a matrix the same size as mat, 1s where the matrix element is contained in the set
	OUTPUTS: None 
*/
Mat is_member(const Mat mat, const Mat set)
{
	CV_Assert(set.rows == 1);
	CV_Assert(set.type() == mat.type());
	Mat ismember(mat.size(), CV_32S, Scalar(0));
	int flag_continue = 0;
	if (mfp(mat))
	{
		// iterate through matrix
		for (int r = 0; r < mat.rows; r++)
		{
			for (int c = 0; c < mat.cols; c++)
			{
				// iterate through set
				for (int s = 0; s < set.cols; s++)
				{
					if (mat.at<double>(r, c) == set.at<double>(s))
					{
						//element is contained in set
						ismember.at<int>(r, c) = 1;
						flag_continue = 1;
						break;
					}
				}
				if (flag_continue)
				{
					flag_continue = 0;
					continue;
				}
			}
		}
	}
	else
	{
		// iterate through matrix
		for (int r = 0; r < mat.rows; r++)
		{
			for (int c = 0; c < mat.cols; c++)
			{
				// iterate through set
				for (int s = 0; s < set.cols; s++)
				{
					if (mat.at<int>(r, c) == set.at<int>(s))
					{
						//element is contained in set
						ismember.at<int>(r, c) = 1;
						flag_continue = 1;
						break;
					}
				}
				if (flag_continue)
				{
					flag_continue = 0;
					continue;
				}
			}
		}
	}
	return ismember;
}

/*
	show whether elements in matrix are NOT contained in a set 

	INPUTS: mat - the input matrix whose values we'll check
			src - the set of values to check for
	RETURNS: a matrix the same size as mat, 1s where the matrix element is NOT contained in the set
	OUTPUTS: None 
*/
Mat is_not_member(const Mat mat, const Mat set)
{
	Mat isnotmember = is_member(mat, set);

	int L = isnotmember.rows*isnotmember.cols;
	for (int i = 0; i < L; i++)
	{
		isnotmember.at<int>(i) = !isnotmember.at<int>(i);
	}

	return isnotmember;
}

/* 
	create a larger matrix by combining a series of matrices horizontally
	Because this function uses a variable argument list, we can't just use the Mat type by itself.
	Instead, we must use pointers to Mats.

	INPUTS: numMats - the number of matrices used in the concatenation
			dest - the matrix that will hold the result
			... - a list of Mat* (pointers to Mats) [THEY MUST HAVE THE SAME NUMBER OF ROWS]
	RETURNS: None
	OUTPUTS: dest - the matrix that is the result of the concatenation
*/
void horiz_concat(int numMats, Mat & dest, ...)
{
	va_list list;
	va_start(list, dest);

	/* initially push list of Mat pointers to a vector */
	std::vector<Mat*> mats;
	for (int n = 0; n < numMats; n++)
	{
		Mat* cindy = va_arg(list, Mat*);
		if (cindy->rows != 0 && cindy->cols != 0)
			mats.push_back(cindy);
	}

	Mat temp;
	if (!mats.empty())
	{
		// cindy points to our next Mat to concat
		Mat * cindy = mats[0];
		int rows = cindy->rows;
		int type = cindy->type();
		temp = cindy->clone();
		for (unsigned int n = 1; n < mats.size(); n++)
		{
			cindy = mats[n];
			CV_Assert(cindy->rows == rows && cindy->type() == type);
			hconcat(temp, *cindy, temp);
		}
	}

	va_end(list);

	dest = temp.clone();
}

/* 
	create a larger matrix by combining a series of matrices horizontally
	Because this function uses a variable argument list, we can't just use the Mat type by itself.
	Instead, we must use pointers to Mats.

	INPUTS: numMats - the number of matrices used in the concatenation
			... - a list of Mat* (pointers to Mats) [THEY MUST HAVE THE SAME NUMBER OF ROWS]
	RETURNS: the matrix that is the result of the concatenation
	OUTPUTS: None
*/
Mat horiz_concat(int numMats, ...)
{
	va_list list;
	va_start(list, numMats);

	/* initially push list of Mat pointers to a vector */
	std::vector<Mat*> mats;
	for (int n = 0; n < numMats; n++)
	{
		Mat* cindy = va_arg(list, Mat*);
		if (cindy->rows != 0 && cindy->cols != 0)
			mats.push_back(cindy);
	}

	Mat temp;
	if (!mats.empty())
	{
		// cindy points to our next Mat to concat
		Mat * cindy = mats[0];
		int rows = cindy->rows;
		int type = cindy->type();
		temp = cindy->clone();
		for (unsigned int n = 1; n < mats.size(); n++)
		{
			cindy = mats[n];
			CV_Assert(cindy->rows == rows && cindy->type() == type);
			hconcat(temp, *cindy, temp);
		}
	}

	va_end(list);

	return temp;
}

/* 
	create a larger matrix by combining a series of matrices vertically
	Because this function uses a variable argument list, we can't just use the Mat type by itself.
	Instead, we must use pointers to Mats.

	INPUTS: numMats - the number of matrices used in the concatenation
			dest - the matrix that will hold the result
			... - a list of Mat* (pointers to Mats) [THEY MUST HAVE THE SAME NUMBER OF COLUMNS]
	RETURNS: None
	OUTPUTS: dest - the matrix that is the result of the concatenation
*/
void vert_concat(int numMats, Mat & dest, ...)
{
	va_list list;
	va_start(list, dest);

	/* initially push list of Mat pointers to a vector */
	std::vector<Mat*> mats;
	for (int n = 0; n < numMats; n++)
	{
		Mat* cindy = va_arg(list, Mat*);
		if (cindy->rows != 0 && cindy->cols != 0)
			mats.push_back(cindy);
	}

	Mat temp;
	if (!mats.empty())
	{
		// cindy points to our next Mat to concat
		Mat * cindy = mats[0];
		int cols = cindy->cols;
		int type = cindy->type();

		temp = cindy->clone();
		for (unsigned int n = 1; n < mats.size(); n++)
		{
			cindy = mats[n];
			CV_Assert(cindy->cols == cols && cindy->type() == type);
			vconcat(temp, *cindy, temp);
		}
	}

	va_end(list);

	dest = temp.clone();
}

/* 
	create a larger matrix by combining a series of matrices vertically
	Because this function uses a variable argument list, we can't just use the Mat type by itself.
	Instead, we must use pointers to Mats.

	INPUTS: numMats - the number of matrices used in the concatenation
			... - a list of Mat* (pointers to Mats) [THEY MUST HAVE THE SAME NUMBER OF COLUMNS]
	RETURNS: the matrix that is the result of the concatenation
	OUTPUTS: None
*/
Mat vert_concat(int numMats, ...)
{
	va_list list;
	va_start(list, numMats);

	/* initially push list of Mat pointers to a vector */
	std::vector<Mat*> mats;
	for (int n = 0; n < numMats; n++)
	{
		Mat* cindy = va_arg(list, Mat*);
		if (cindy->rows != 0 && cindy->cols != 0)
			mats.push_back(cindy);
	}

	Mat temp;
	if (!mats.empty())
	{
		// cindy points to our next Mat to concat
		Mat * cindy = mats[0];
		int cols = cindy->cols;
		int type = cindy->type();

		temp = cindy->clone();
		for (unsigned int n = 1; n < mats.size(); n++)
		{
			cindy = mats[n];
			CV_Assert(cindy->cols == cols && cindy->type() == type);
			vconcat(temp, *cindy, temp);
		}
	}

	va_end(list);

	return temp;
}

/* 
	create a larger matrix by combining a series of matrices
	Because this function uses a variable argument list, we can't just use the Mat type by itself.
	Instead, we must use pointers to Mats.

	INPUTS: rows - the number of matrices down (not actually rows)
			cols - the number of matrices across (not actually columns)
				(sorry for the misleading variable names)
			dest - the matrix that will hold the result
			... - a list of Mat* (pointers to Mats)
					every "row" of matrices must agree in their number of rows
					every "column" of matrices must agree in their number of columns
	RETURNS: None
	OUTPUTS: dest - the matrix that is the result of the concatenation

	EXAMPLE:
		Mat a, b, c, d;
		//	assign values to a, b, c, and d
		//	a is 3x2, b is 3x1, c is 1x2, d is 1x1
		//	(notice that a and b agree in number of rows, b and d agree in number of cols, and so on...)
		Mat e;
		mat_concat(2, 2, e, &a, &b, &c, &d);

		// e is a 4x3 matrix
		// e = [ a b ]
		//     [ c d ]
*/
void mat_concat(int rows, int cols, Mat & dest, ...)
{
	va_list list;
	va_start(list, dest);

	std::vector< std::vector<Mat*> > mats;

	for (int r = 0; r < rows; r++)
	{
		std::vector<Mat*> tempRowVec;
		for (int c = 0; c < cols; c++)
		{
			Mat* cindy = va_arg(list, Mat*);
			tempRowVec.push_back(cindy);
		}
		mats.push_back(tempRowVec);
	}

	int r = 0;
	Mat matTemp;
	do{
		Mat * cindy = mats[r][0];
		int rows = cindy->rows;
		int type = cindy->type();
		Mat temp = cindy->clone();
		for (int c = 1; c < cols; c++)
		{
			cindy = mats[r][c];
			CV_Assert(cindy->rows == rows && cindy->type() == type);
			hconcat(temp, *cindy, temp);
		}

		if (matTemp.dims == 0)
			temp.copyTo(matTemp);
		else
			vconcat(matTemp, temp, matTemp);

		r++;
	} while (r < rows);

	dest = matTemp.clone();
}
 
/* 
	create a larger matrix by combining a series of matrices
	Because this function uses a variable argument list, we can't just use the Mat type by itself.
	Instead, we must use pointers to Mats.

	INPUTS: rows - the number of matrices down (not actually rows)
			cols - the number of matrices across (not actually columns)
				(sorry for the misleading variable names)
			... - a list of Mat* (pointers to Mats)
					every "row" of matrices must agree in their number of rows
					every "column" of matrices must agree in their number of columns
	RETURNS: the matrix that is the result of the concatenation
	OUTPUTS: None

	EXAMPLE:
		Mat a, b, c, d;
		//	assign values to a, b, c, and d
		//	a is 3x2, b is 3x1, c is 1x2, d is 1x1
		//	(notice that a and b agree in number of rows, b and d agree in number of cols, and so on...)
		Mat e = mat_concat(2, 2, &a, &b, &c, &d);

		// e is a 4x3 matrix
		// e = [ a b ]
		//     [ c d ]
*/
Mat mat_concat(int rows, int cols, ...)
{
	va_list list;
	va_start(list, cols);

	std::vector< std::vector<Mat*> > mats;

	for (int r = 0; r < rows; r++)
	{
		std::vector<Mat*> tempRowVec;
		for (int c = 0; c < cols; c++)
		{
			Mat* cindy = va_arg(list, Mat*);
			tempRowVec.push_back(cindy);
		}
		mats.push_back(tempRowVec);
	}

	int r = 0;
	Mat matTemp;
	do{
		Mat * cindy = mats[r][0];
		int rows = cindy->rows;
		int type = cindy->type();
		Mat temp = cindy->clone();
		for (int c = 1; c < cols; c++)
		{
			cindy = mats[r][c];
			CV_Assert(cindy->rows == rows && cindy->type() == type);
			hconcat(temp, *cindy, temp);
		}

		if (matTemp.dims == 0)
			temp.copyTo(matTemp);
		else
			vconcat(matTemp, temp, matTemp);

		r++;
	} while (r < rows);

	return matTemp;
}

/*
	check whether two matrices are equivalent
	by checking that each of their values match

	INPUTS: mat1 - first matrix to compare
			mat2 - second matrix to compare
	RETURNS: 1 indicating equivalency, 0 indicating not equal
	OUTPUTS: None
*/
int check_equal(const Mat mat1, const Mat mat2)
{
	CV_Assert(mat1.size() == mat2.size());
	CV_Assert(mat1.type() == mat2.type());

	if (mfp(mat1))
	{
		for (int r = 0; r < mat1.rows; r++)
		{
			for (int c = 0; c < mat1.cols; c++)
			{
				if (mat1.at<double>(r,c) != mat2.at<double>(r,c))
					return 0;
			}
		}
	}
	else
	{
		for (int r = 0; r < mat1.rows; r++)
		{
			for (int c = 0; c < mat1.cols; c++)
			{
				if (mat1.at<int>(r,c) != mat2.at<int>(r,c))
					return 0;
			}
		}
	}
	return 1;
}

/*
	assign a matrix to have the same size, type, and values as the source matrix
	
	INPUTS:	dest - the destination matrix that we will copy to
			src - the source matrix that we will copy from
	RETURNS: None
	OUTPUTS: dest - the newly copied matrix
*/
void assign_mat(Mat & dest, const Mat src)
{
	src.copyTo(dest);
}

/*
	assign a matrix to have the same size, type, and values as the source matrix
	
	INPUTS:	dest - the destination matrix that we will copy to
			src - the result of a matrix expression that we will copy from
	RETURNS: None
	OUTPUTS: dest - the newly copied matrix
*/
void assign_mat(Mat & dest, const MatExpr src)
{
	dest = src;
}

/*
	assigns values in destination matrix to match particular values (defined by ranges) in the source matrix

	This function performs the actual resizing (if necessary) of the destination matrix. We call this from
	another overloaded function (below). This function should not be used directly. Why? Well, lets be honest:
	Mat::Ranges are kind of a pain to work with. It's more convenient to put in actual numbers.
	(See next function to see what I mean.)

	INPUTS: dest - the destination matrix that we will copy to
			src - the source matrix that we will copy from
			dRowRange - Mat::Range defining the range of rows that will be changed in dest
			dColRange - Mat::Range defining the range of columns that will be changed in dest
			sRowRange - Mat::Range defining the range of rows that will be copied from src
			sColRange - Mat::Range defining the range of columns that will be copied from src
	RETURNS: None
	OUTPUTS: dest - the modified matrix, with values copied from src matrix
*/
void assign_mat(Mat & dest, const Mat src, Range dRowRange, Range dColRange, Range sRowRange, Range sColRange)
{
	int dRowAll = 0;
	int dColAll = 0;
	int sRowAll = 0;
	int sColAll = 0;
	if (dRowRange == Range::all()) dRowAll = 1;
	if (dColRange == Range::all()) dColAll = 1;
	if (sRowRange == Range::all()) sRowAll = 1;
	if (sColRange == Range::all()) sColAll = 1;

	if (sRowAll)
	{
		sRowRange.start = 0;
		sRowRange.end = src.rows;
	}
	if (sColAll)
	{
		sColRange.start = 0;
		sColRange.end = src.cols;
	}

	if (dest.dims == 0 || (dest.rows == 0 && dest.cols == 0))
	{
		dest = Mat(1, 1, src.type(), Scalar(0));
		if (dRowAll)
		{
			dRowRange.start = 0;
			dRowRange.end = sRowRange.end - sRowRange.start;
		}
		if (dColAll)
		{
			dColRange.start = 0;
			dColRange.end = sColRange.end - sColRange.start;
		}
	}
	else
	{
		if (dRowAll)
		{
			dRowRange.start = 0;
			dRowRange.end = dest.rows;
		}
		if (dColAll)
		{
			dColRange.start = 0;
			dColRange.end = dest.cols;
		}
	}

	if (dRowRange.end > dest.rows)
	{
		dest.resize(dRowRange.end, Scalar(0));
	}
	if (dColRange.end > dest.cols)
	{
		Mat zeros = Mat::zeros(dest.rows, dColRange.end - dest.cols, dest.type());
		horiz_concat(2, dest, &dest, &zeros);
	}

	CV_Assert((dRowRange.end - dRowRange.start) == (sRowRange.end - sRowRange.start));
	CV_Assert((dColRange.end - dColRange.start) == (sColRange.end - sColRange.start));

	src(sRowRange, sColRange).copyTo(dest(dRowRange, dColRange));
}

/*
	assigns values in destination matrix to match particular values (defined by ranges) in the source matrix

	This function is recommended for use when dealing with copying ranges of matrices to other matrices.
	For ranges of rows and columns, it uses INCLUSIVE indices for both start and end. This is in contrast to an
	INCLUSIVE index for start and an EXCLUSIVE index for end. Cool, huh? (Yeah, I know. It's not THAT cool.)

	INPUTS: dest - the destination matrix that we will copy to
			src - the source matrix that we will copy from
			dRowSt - the FIRST row to copy to
			dRowEn - the LAST row to copy to (notice this is different from how Mat::Range works)
			dColSt - the FIRST col to copy to
			dColEn - the LAST col to copy to (notice this is different from how Mat::Range works)
			sRowSt - the FIRST row to copy from
			sRowEn - the LAST row to copy from (notice this is different from how Mat::Range works)
			sColSt - the FIRST col to copy from
			sColEn - the LAST col to copy from (notice this is different from how Mat::Range works)
	RETURNS: None
	OUTPUTS: dest - the modified matrix, with values copied from src matrix

	I know what you're thinking: "But what if I want to copy ALL rows and only CERTAIN columns?"
	Easy!
	You can specify this using -1 for for BOTH START AND END in the range.

	EXAMPLE:
		copy columns 2 through 5 of a 3x8 matrix to a 6x10 matrix (kind of a screwy example, but bear with me)
		Let's say A is the 3x8 and B is the 6x10
		We'll copy these columns to the top left corner of B
	
		In matlab, you'd do
			B(1:3, 1:4) = A(:, 2:5);

		Using this function in C++
			assign_mat(B, A, 0, 2, 0, 3, -1, -1, 1, 4);
		
		Did you follow that? Simple, huh? :)
*/
void assign_mat(Mat & dest, const Mat src, int dRowSt, int dRowEn, int dColSt, int dColEn, int sRowSt, int sRowEn, int sColSt, int sColEn)
{
	CV_Assert(dRowSt <= dRowEn && dColSt <= dColEn && sRowSt <= sRowEn && sColSt <= sColEn);

	Range dRowRange, dColRange, sRowRange, sColRange;
	if (dRowSt == -1 && dRowEn == -1)
		dRowRange = Range::all();
	else dRowRange = Range(dRowSt, dRowEn+1); // +1 cause Mat::Range is defined differently

	if (dColSt == -1 && dColEn == -1)
		dColRange = Range::all();
	else dColRange = Range(dColSt, dColEn+1);
	
	if (sRowSt == -1 && sRowEn == -1)
		sRowRange = Range::all();
	else sRowRange = Range(sRowSt, sRowEn+1);

	if (sColSt == -1 && sColEn == -1)
		sColRange = Range::all();
	else sColRange = Range(sColSt, sColEn+1);

	// calls previous function
	assign_mat(dest, src, dRowRange, dColRange, sRowRange, sColRange);
}

/*
	assign values in a matrix's row(s) to the values in another matrix's row(s)

	INPUTS: dest - the destination matrix that we will copy to
			src - the source matrix that we will copy from
			dRowSt - the FIRST row to copy to
			dRowEn - the LAST row to copy to (notice this is different from how Mat::Range works)
			sRowSt - the FIRST row to copy from
			sRowEn - the LAST row to copy from (notice this is different from how Mat::Range works)
	RETURNS: None
	OUTPUTS: dest - the modified matrix, with values copied from src matrix
*/
void assign_row(Mat & dest, const Mat src, int dRowSt, int dRowEn, int sRowSt, int sRowEn)
{
	assign_mat(dest, src, dRowSt, dRowEn, -1, -1, sRowSt, sRowEn, -1, -1);
}

/*
	assign values in a matrix's column(s) to the values in another matrix's column(s)

	INPUTS: dest - the destination matrix that we will copy to
			src - the source matrix that we will copy from
			dColSt - the FIRST col to copy to
			dColEn - the LAST col to copy to (notice this is different from how Mat::Range works)
			sColSt - the FIRST col to copy from
			sColEn - the LAST col to copy from (notice this is different from how Mat::Range works)
	RETURNS: None
	OUTPUTS: dest - the modified matrix, with values copied from src matrix
*/
void assign_col(Mat & dest, const Mat src, int dColSt, int dColEn, int sColSt, int sColEn)
{
	assign_mat(dest, src, -1, -1, dColSt, dColEn, -1, -1, sColSt, sColEn);
}

/*
	assigns a value of type double to a single element in a matrix
	
	INPUTS: dest - the destination matrix that we will insert the new value into
			k - the value (type double) to insert
			row and col - the location where the new value will be inserted 
	RETURNS: None
	OUTPUTS: dest - the modified matrix, with the new value in place
*/
void assign_elem_double(Mat & dest, double k, int row, int col)
{
	Mat kMat = (Mat_<double>(1,1) << k); // make a 1 element matrix [k]
	assign_mat(dest, kMat, row, row, col, col, 0, 0, 0, 0);
}

/*
	assigns a value of type int to a single element in a matrix
	
	INPUTS: dest - the destination matrix that we will insert the new value into
			k - the value (type int) to insert
			row and col - the location where the new value will be inserted 
	RETURNS: None
	OUTPUTS: dest - the modified matrix, with the new value in place
*/
void assign_elem_int(Mat & dest, int k, int row, int col)
{
	Mat kMat = (Mat_<int>(1,1) << k); // make a 1 element matrix [k]
	assign_mat(dest, kMat, row, row, col, col, 0, 0, 0, 0);
}

/*
	finds the maximum value in a matrix of floating point type 

	INPUTS: src - the input matrix
	RETURNS: the maximum value in the matrix
	OUTPUTS: None
*/
double max_double(const Mat src)
{
	double max = 0;
	for (int i = 0; i < src.rows * src.cols; i++)
	{
		double carl = src.at<double>(i);
		if (max < carl)
			max = carl;
	}

	return max;
}

/*
	finds the maximum value in a matrix of floating point type 

	INPUTS: src - the input matrix
	RETURNS: the maximum value in the matrix
	OUTPUTS: idxOfMax - index of the maximum value
*/
double max_double(const Mat src, int & idxOfMax)
{
	double max = 0;
	idxOfMax = 0;
	for (int i = 0; i < src.rows * src.cols; i++)
	{
		double carl = src.at<double>(i);
		if (max < carl)
		{
			idxOfMax = i;
			max = carl;
		}
	}

	return max;
}

/*
	finds the maximum value in a matrix of integer type 

	INPUTS: src - the input matrix
	RETURNS: the maximum value in the matrix
	OUTPUTS: None
*/
int max_int(const Mat src)
{
	int max = -1.0/0.0; // -inf
	for (int i = 0; i < src.rows * src.cols; i++)
	{
		int carl = src.at<int>(i);
		if (max < carl)
			max = carl;
	}

	return max;
}

/*
	finds the mean of a matrix of floating point type along an axis

	INPUTS: src - the input matrix
			axis - 0 means take mean across rows, 1 means take mean across columns
	RETURNS: the mean of the matrix
	OUTPUTS: None

	EXAMPLE
			a = [ 1  2  3  4
				  5  6  7  8
				  9 10 11 12
				 13 14 15 16 ]
		mean(a, 0) gives [ 2.5 6.5 10.5 14.5 ]
		mean(a, 1) gives [ 7; 8; 9; 10 ]
*/
Mat mean_double(const Mat src, const int axis)
{
	Mat sum, mean;
	if (axis == 0)
	{
		sum = Mat::zeros(1, src.cols, CV_64F);
		for (int i = 0; i < src.rows; i++)
		{
			Range rowRange(i, i+1);
			Range colRange(0, src.cols);
			sum += src(rowRange, colRange);
		}
		mean = sum/(double)src.rows;
	}
	if (axis == 1)
	{
		sum = Mat::zeros(src.rows, 1, CV_64F);
		for (int i = 0; i < src.cols; i++)
		{
			Range rowRange(0, src.rows);
			Range colRange(i, i+1);
			sum += src(rowRange, colRange);
		}
		mean = sum/(double)src.cols;
	}

	return mean;
}

/*
	creates a range of values, similar to a:b in matlab, where a and b are integers

	INPUTS: start - the first value
			end - the last value
	RETURNS: a matrix (column vector) that holds the range of values (type double)
	OUTPUTS: None
*/
Mat range_double(int start, int end)
{
	CV_Assert(start <= end);
	Mat range(1, 1, CV_64F);
	range.ATD(0,0) = (double)start;
	int size = 1;
	for (int i = start + 1; i <= end; i++, size++)
	{
		assign_elem_double(range, (double)i, 0, size);
	}
	
	return range;
}

/*
	creates a range of values, similar to a:b in matlab, where a and b are integers

	INPUTS: start - the first value
			end - the last value
	RETURNS: a matrix (column vector) that holds the range of values (type int)
	OUTPUTS: None
*/
Mat range_int(int start, int end)
{
	CV_Assert(start <= end);
	Mat range(1, 1, CV_32S);
	range.at<int>(0,0) = start;
	int size = 1;
	for (int i = start + 1; i <= end; i++, size++)
	{
		assign_elem_int(range, (double)i, 0, size);
	}
	
	return range;
}

/*
	similar to m(a, b) in Matlab, where m is a matrix, a is a list of rows indice, b a list of column indices
	To do m(:, c), pass in an empty matrix (e.g. Mat()) for r, and vice versa for m(r, :)

	INPUTS: src - the matrix that we'll pick values from
			r - a column vector containing the rows of the values we want
			c - a column vector containing the columns of the values we want
	RETURNS: The matrix formed from src as a result of the choices for r and c
	OUTPUTS: None
*/
Mat mat_from_indices(const Mat src, const Mat r, const Mat c)
{
	Mat dest;
	if (r.dims == 0 && c.dims == 0)
	{
		assign_mat(dest, src);
	}
	else if (r.dims == 0)	/* all rows */
	{
		CV_Assert(c.cols == 1);
		for (int i = 0; i < c.rows; i++)
		{
			int col_idx = c.at<int>(i);
			CV_Assert(col_idx < src.cols);
			assign_col(dest, src, i, i, col_idx, col_idx);
		}
	}
	else if (c.dims == 0)
	{
		CV_Assert(r.cols == 1);
		for (int i = 0; i < r.rows; i++)
		{
			int row_idx = r.at<int>(i);
			CV_Assert(row_idx < src.rows);
			assign_row(dest, src, i, i, row_idx, row_idx);
		}
	}
	else
	{
		Mat temp;
		for (int a = 0; a < c.rows; a++)
		{
			Mat col = r.row(0); /* temp for the c-th column */
			for (int b = 0; b < r.rows; b++)
			{
				int row_idx = r.at<int>(b);
				CV_Assert(row_idx < src.rows);
				assign_row(temp, src, b, b, row_idx, row_idx);
			}
			int col_idx = c.at<int>(a);
			CV_Assert(col_idx < src.cols);
			assign_col(dest, temp, a, a, col_idx, col_idx);
		}
	}
	return dest;
}

/*
	given a list containing 0s or positive integers, select the rows corresponding to positive ints
	this forms a matrix from the selected rows

	INPUTS: src - the matrix we'll pick rows from
			r - a matrix containing the indices of rows to pick
				(r doesn't necessarily have to be a vector, but using non-vectors is untested)
	RETURNS: the matrix formed from src as a result of the choice for r
	OUTPUTS: None

	EXAMPLE:
		src =	[4 0]			
				[1 2]

		r = [0 1]

		So, select_rows(src, r) would return [1 2]
*/
Mat select_rows(const Mat src, const Mat r)
{
	Mat dest;
	if (r.dims == 0)
	{
		assign_mat(dest, src); // if list r is empty
	}
	else
	{
		CV_Assert(r.rows * r.cols == src.rows); // must be same number of rows in r as there are rows in src
		int cnt = 0;
		for (int i = 0; i < r.rows * r.cols; i++)
		{
			if (r.at<int>(i))
			{
				assign_row(dest, src, cnt, cnt, i, i);
				cnt++;
			}
		}
	}
	return dest;
}

/*
	given an integer, output the corresponding row from the src matrix

	INPUTS: src - the matrix we'll pick rows from
			row - the number of the row to output
	RETURNS: the matrix formed from src as a result of the choice for row
	OUTPUTS: None

	EXAMPLE:
		src =	[4 0]			
				[1 2]

		row = [0 1]

		So, select_rows(src, r) would return [1 2]
*/
//TODO Mat select_row(const Mat src, const Mat r);

/*
	given a list containing 0s or positive integers, select the columns corresponding to positive ints
	this forms a matrix from the selected columns

	INPUTS: src - the matrix we'll pick columns from
			c - a matrix containing the indices of columns to pick
				(c doesn't necessarily have to be a vector, but using non-vectors is untested)
	RETURNS: the matrix formed from src as a result of the choice for c
	OUTPUTS: None

	EXAMPLE:
		src =	[4 0]			
				[1 2]

		c = [0 1]

		So, select_rows(src, c) would return [0; 2]
*/
Mat select_cols(const Mat src, const Mat c)
{
	Mat dest;
	if (c.dims == 0)
	{
		assign_mat(dest, src); // if list r is empty
	}
	else
	{
		CV_Assert(c.rows * c.cols == src.cols); // must be same number of elems in c as there are cols in src
		int cnt = 0;
		for (int i = 0; i < c.rows * c.cols; i++)
		{
			if (c.at<int>(i))
			{
				assign_col(dest, src, cnt, cnt, i, i);
				cnt++;
			}
		}
	}
	return dest;
}

Mat vector_to_Mat_double(const std::vector<double> v)
{
	Mat mat = Mat::zeros(v.size(), 1, CV_64F);
	for (int i = 0; i < v.size(); i++)
	{
		mat.ATD(i, 0) = v[i];
	}
	return mat;
}

std::vector<double> Mat_to_vector_double(const cv::Mat mat)
{
	std::vector<double> v;
	for (int i = 0; i < mat.total(); i++)
	{
		v.push_back(mat.at<double>(i));
	}
	return v;
}

// Writes to file in a pretty, human-readable format
void write_mat_to_file(std::ofstream & f, Mat mat)
{
	f << "[";
	int r = mat.rows;
	int c = mat.cols;
	f << std::fixed;
	for (int j = 0; j < r; j++)
	{
		for (int i = 0; i < c; i++)
		{
			if (mat.type() == CV_64F)
			{
				double x = mat.at<double>(j, i);
				if (x < 0)
				{
					f << std::setprecision(9) << x;
				}
				else
				{
					f << std::setprecision(10) << x;
				}
			}
			else
			{
				int x = mat.at<int>(j, i);
				f << x;
			}
				
			if (i == c - 1)
			{
				if (j == r - 1)
				{
					f << "]";
				}
				else
				{
					f << ";\n";
				}
			}
			else
			{
				f << ", ";
			}
		}
	}
}

// Writes to file in a CSV format
void write_mat_to_csv(std::ofstream & f, Mat mat)
{
	int r = mat.rows;
	int c = mat.cols;
	for (int j = 0; j < r; j++)
	{
		for (int i = 0; i < c; i++)
		{
			if (mat.type() == CV_64F)
			{
				double x = mat.at<double>(j, i);
				f << x;
			}
			else
			{
				int x = mat.at<int>(j, i);
				f << x;
			}
				
			if (i == c - 1)
			{
				if (j == r - 1)
				{
					// nothing
				}
				else
				{
					f << "\n";
				}
			}
			else
			{
				f << ", ";
			}
		}
	}
}

// Assuming file is csv
Mat read_mat_from_file(const std::string filename)
{
	std::string line;
	std::ifstream csv(filename.c_str());
	Mat output;
	if (csv.is_open())
	{
		int r = 0;
		while ( getline(csv, line) )
		{
			std::stringstream allTokens(line);
			std::string token;
			int c = 0;
			while ( getline(allTokens, token, ',') )
			{
				double value;
				sscanf(token.c_str(), "%lf", &value);
				// probably super inefficient, but fuggit
				assign_elem_double(output, value, r, c);
				c++;
			}
			r++;
		}
	}
	else
	{
		std::cout << "ERROR: read_mat_from_file: can't find file" << std::endl;
	}
	return output;
}

Mat linspace(double a, double b, int n)
{
	std::vector<double> array;
	double step = (b - a)/(n - 1);

	while (a <= b)
	{
		array.push_back(a);
		a += step;
	}
	return vector_to_Mat_double(array);
}

/*
template <class T>
std::string to_str(T t)
{
	std::ostringstream convert;
	convert << t;
	return convert.str();
}
*/

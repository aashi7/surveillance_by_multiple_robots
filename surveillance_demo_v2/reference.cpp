// MEX function to test data import from Matlab to C++ and export back
// for multi dimensional double and character arrays
// ------------------------------------------------------------------------
// Compile in Matlab with: -mex filename.cpp
// Call in Matlab with: [DoubleArray_OUT CharArray_OUT] = imexport(DoubleArray_IN CharArray_IN)
// ========================================================================
#include <cstring>   // memcpy
#include <math.h>
#include "matrix.h"  // mwSize, mwIndex
#include "mex.h"
#define IS_REAL_FULL_DOUBLE(P) (!mxIsComplex(P) && !mxIsSparse(P) && mxIsDouble(P))
#define IS_REAL_FULL_CHAR(P) (mxIsChar(P) && !mxIsSparse(P)) 
void mexFunction(int nlhs, mxArray *plhs[],  // plhs: array of mxArray pointers to output
      int nrhs, const mxArray *prhs[])     // prhs: array of mxArray pointers to input
{
  // Macros for the double ouput and input arguments:
  #define double_IN  prhs[0]   // 3-dim double array IN
  #define double_OUT plhs[0]   // 3-dim double array OUT
  // Macros for the char ouput and input arguments:
  #define char_IN  prhs[1]     // 3-dim double array IN
  #define char_OUT plhs[1]     // 3-dim double array OUT 
    // Check number of arguments:
    if (nrhs<2 || nrhs>3)
        mexErrMsgTxt("Error: Two input arguments required.");
    else if (nrhs<2 || nrhs>3)
        mexErrMsgTxt("Internal error: Wrong number of output arguments.");
    // Check type of arguments:
    if (!IS_REAL_FULL_DOUBLE(double_IN))
        mexErrMsgTxt("Error: First input argument must be a double array.");    
    else if (!IS_REAL_FULL_CHAR(char_IN))
        mexErrMsgTxt("Error: Second input argument must be a char array.");
    // Declaration for double arrays:
    mwSize iter_doubleIN;        // number of dimensions of input
    const mwSize *dims_doubleIN; // dimensions of input
    mxArray *p_doubleIN;         // pointer to input array  
    mxArray *p_doubleOUT;        // pointer to output array
    int bytes_to_copy_double, iter_double;  
    // int dim1, dim2, dim3;       // not needed when memcpy is used    
    // Declaration for char arrays:
    mwSize iter_charIN;          // number of dimensions of input 
    const mwSize *dims_charIN;   // dimensions of input
    mxChar *p_charIN;            // pointers to char input array
    mxChar *p_charOUT;           // pointer to char output array
    int bytes_to_copy_char, iter_char; 
    // size_t dim1, dim2, dim3;    // not needed when memcpy is used       
    // Initialization for double arrays: 
    iter_doubleIN = mxGetNumberOfDimensions(double_IN);  // get # of dims
    dims_doubleIN = mxGetDimensions(double_IN);          // get dimensions
    p_doubleIN = (mxArray *)mxGetPr(double_IN);          // pointer to input      
    double_OUT = mxCreateNumericArray(iter_doubleIN,dims_doubleIN,mxDOUBLE_CLASS,mxREAL);  // Create an N[0]×...× N[K-1] array for output
    p_doubleOUT = (mxArray *)mxGetPr(double_OUT);        // pointer to output 
    // Initialization for char arrays: 
    iter_charIN = mxGetNumberOfDimensions(char_IN);      // get # of dims
    dims_charIN = mxGetDimensions(char_IN);              // get dimensions    
    p_charIN = (mxChar *)mxGetData(char_IN);             // pointer to input
    char_OUT = mxCreateCharArray(iter_charIN,dims_charIN);  // Create N[0]×...×N[K-1] char array for output
    p_charOUT = (mxChar *)mxGetData(char_OUT);           // pointer to output
    // DOUBLE - Copy data into the output array:
    bytes_to_copy_double = 1;
    for(iter_double=0; iter_double < iter_doubleIN; iter_double++)
    {
        bytes_to_copy_double = bytes_to_copy_double*dims_doubleIN[iter_double];
    }
    bytes_to_copy_double = bytes_to_copy_double * sizeof(double);  // total size of bytes to be copied
    memcpy( (void *)p_doubleOUT, (void *)p_doubleIN, bytes_to_copy_double );  // void * memcpy ( void * destination, const void * source, size_t num );
    // CHAR - Copy data into the output array:
    bytes_to_copy_char = 1;
    for(iter_char=0; iter_char < iter_charIN; iter_char++)
    {
        bytes_to_copy_char = bytes_to_copy_char*dims_charIN[iter_char];
    }
    bytes_to_copy_char = bytes_to_copy_char * 2;  // *2 because Matlab stores each char data as 2-bytes instead of 1-byte as in C
    memcpy( (void *)p_charOUT, (void *)p_charIN, bytes_to_copy_char );  // void * memcpy ( void * destination, const void * source, size_t num );
    /* Alternative to copy data into the char output array(less efficient):
    for(dim3 = 0; dim3 < dims_charIN[2]; dim3++) // Compute a matrix with normalized columns 
    {
        for(dim2 = 0; dim2 < dims_charIN[1]; dim2++)
        {
            for(dim1 = 0; dim1 < dims_charIN[0]; dim1++)
            {               
                  p_charOUT[dim3+dim2*dims_charIN[2]+dim1*dims_charIN[2]*dims_charIN[1]] = 
                        p_charIN[dim3+dim2*dims_charIN[2]+dim1*dims_charIN[2]*dims_charIN[1]];
            }
        }
    }  */  
    /* Alternative to copy data into the double output mxArray (less efficient):
    for(dim3 = 0; dim3 <= dims_doubleIN[2]; dim3++) 
    {
        mexPrintf("dim3 = %d \n",dim3);
        for(dim2 = 0; dim2 <= dims_doubleIN[1]; dim2++)
        {
            mexPrintf("dim2 = %d \n",dim2);
            for(dim1 = 0; dim1 <= dims_doubleIN[0]; dim1++)
            {               
                mexPrintf("dim1 = %d \n",dim1);
                mexPrintf("element %d \n",(dim1-1)+(dim2-1)*(dims_doubleIN[0])+(dim3-1)*(dims_doubleIN[0])*(dims_doubleIN[1]));
                p_doubleOUT[(dim3)+(dim2)*(dims_doubleIN[2])+(dim1)*(dims_doubleIN[2])*(dims_doubleIN[1])] = 
                        p_doubleIN[(dim3)+(dim2)*(dims_doubleIN[2])+(dim1)*(dims_doubleIN[2])*(dims_doubleIN[1])];
                // p_doubleOUT[(dim1-1)+(dim2-1)*(dims_doubleIN[0])+(dim3-1)*(dims_doubleIN[0])*(dims_doubleIN[1])] = p_doubleIN[(dim1-1)+(dim2-1)*(dims_doubleIN[0])+(dim3-1)*(dims_doubleIN[0])*(dims_doubleIN[1])];
            }
        }
    }  */  
    return;
    // Clean up workspace:
    mxDestroyArray(p_doubleIN);  // free memory occupied by input mxArray
    mxDestroyArray(p_doubleOUT); // free memory occupied by output mxArray 
    mxFree(p_charIN);            // free memory occupied by input mxChar
    mxFree(p_charOUT);           // free memory occupied by output mxChar
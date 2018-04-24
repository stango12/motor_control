#include <Python.h>
#include <vector>
#include <iostream>
#include "numpy/arrayobject.h"

using namespace std;

int
main(int argc, char *argv[])
{
	putenv("PYTHONPATH=.");
    PyObject *pName, *pModule, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

    Py_Initialize();
    pName = PyString_FromString("rnn_LSTM_CPU");
    /* Error checking of pName left out */

    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, "predict_connection");
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            //pArgs = PyString_FromString(argv[1]);

            pValue = PyObject_CallObject(pFunc, NULL);
            //Py_DECREF(pArgs);
            if (pValue != NULL) {
                std::vector<int> data;
                if(PyList_Check(pValue))
                {
                    for(Py_ssize_t i = 0; i < PyList_Size(pValue); i++)
                    {
                        PyObject *value = PyList_GetItem(pValue, i);
                        data.push_back(PyFloat_AsDouble(value));
                    }
                }
                for(int j = 0; j < data.size(); j++)
                    std::cout << data[j] << ' ';
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return 1;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", "predict_connection");
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", "rnn_LSTM_CPU");
        return 1;
    }
    Py_Finalize();
    return 0;
}

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Recognizer.h
 * Author: nick
 *
 * Created on April 3, 2016, 9:31 PM
 */

#ifndef RECOGNIZER_H
#define RECOGNIZER_H
#include <cstdlib>

#include <iostream>
#include <fstream>
#include <string>
#include <istream>
#include <sstream>
#include <vector>
#include <cstring>
#include <math.h>
#include <svm.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <mathop.h>
#include <kmeans.h>
//#include <vlad.h>
#include <svm.h>
    

#ifdef __cplusplus
}
#endif
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

using namespace std;

/*
 * 
 */

class recognizer {
public:
    double * codebook;
    double * encoded_features;
    double * bow;
    double * dist;
    double * norm;
    int K, D, num_encoded, dim_encoded, dim_features, n, result;
    double score;
    const char *codebook_file, *encoded_features_file, *norm_file;
    std::string model_files;
    int num_classes;
    svm_model ** models;
    VlKDForest* forest;
    double * probs;
    
    recognizer(const char *a, const char *b, const char *c, std::string d, int e) : codebook_file(a), encoded_features_file(b), norm_file(c), model_files(d), num_classes(e) {

        // Load files
        std::cout << "Hello! My name is 'recognizer'." << std::endl;
        n=1;
        std::cout << "Loading codebook from " << codebook_file << std::endl;
        read_file_legacy(codebook_file, &codebook, &K, &D);
        std::cout << "Loading training data from " << encoded_features_file << std::endl;
        read_file_legacy(encoded_features_file, &encoded_features, &num_encoded, &dim_encoded);
        std::cout << "Loading kernel normalization factor from " << norm_file << std::endl;
        read_file_legacy(norm_file, &norm, &n, &n);   
        std::cout << "Loading SVM models from " << model_files << "*" << std::endl;
        load_models(&models, model_files, num_classes);                
        
        // Initialize structures
        VlKMeans * kmeans = vl_kmeans_new(VL_TYPE_DOUBLE, VlDistanceL2);
        vl_kmeans_set_centers(kmeans, codebook, D, K);
        forest = vl_kdforest_new(VL_TYPE_DOUBLE, D, 1, VlDistanceL2);
        vl_kdforest_build(forest, K, codebook);
        
        probs = (double *) malloc(sizeof(double)*num_classes);
    }
    
    int BagOfWords(int K, int D, int N, double *data, double ** bow){

        int i;
        int *assignments = (int *) malloc(sizeof(int)*N);
        VlKDForestNeighbor d;

        for (i=0; i<N; i++){
            vl_kdforest_query(forest, &d, 1, data+i*D); 
            assignments[i] = d.index;        
        }

        (*bow) = (double*) malloc(sizeof(double)*K);
        memset(*bow, 0, sizeof(double)*K);
        for (i=0; i<N; i++){
    //        printf("%d\n", assignments[i]);
            (*bow)[assignments[i]]++;
        }    

        double ssum = 0;
        for (i=0; i<K; i++){
            ssum += pow((*bow)[i], 2);
        }
        ssum = sqrt(ssum);
        for (i=0; i<K; i++){
            (*bow)[i] /= ssum;
        }
        
        free(assignments);
    //    for (i=0; i<K; i++){
    //        printf("%.8f ", (*bow)[i]);
    //    }
    //    printf("\n");
    //    printf("%.6f\n", (*bow)[0]);
        return 0;
}

    int ChiSquared(int K, int N, double * training, double *testing, double **dist){
        int i;
//        double d;
        (*dist) = (double *) malloc(sizeof(double)*N);
        VlDoubleVectorComparisonFunction dist_fcn = vl_get_vector_comparison_function_d(VlDistanceChi2);

        for (i=0; i<N; i++){
            (*dist)[i] = dist_fcn(K, training+i*K, testing)/2;
    //        printf("%.8f\n", (*kernel)[i]);
        }
        return 0;
    }

    int load_models(svm_model *** models, const string model_files, int num_classes){
        int i;
        string model_file_name;
        (*models) = (svm_model**) malloc(sizeof(svm_model*)*num_classes);
        for (i=0; i<num_classes; i++){
            model_file_name = model_files + SSTR(i+1);
            if(((*models)[i] = svm_load_model(model_file_name.c_str()))==0)
            {
                    fprintf(stderr,"can't open model file %s\n", model_file_name.c_str());
                    exit(1);
            }

        }
        return 0;
    }

    int classify(const double *dist, svm_model ** models, const double norm, const int num_training, const int num_classes, int *result, double *score, double *probs){
    
        int i, cind, max_prob_ind;
        double max_prob;
        double * kernel = (double *) malloc(sizeof(double)*num_training);
        svm_node * kernel_struct = (svm_node*) malloc(sizeof(svm_node)*(num_training+2));
        double p[2] = {0};
        int *label = (int*) malloc(sizeof(int)*2);
//        double * probs = (double *) malloc(sizeof(double)*num_classes);

        // Construct kernel
        kernel_struct[0].index = 0;
        for (i=0; i<num_training; i++){
            kernel[i] = exp(-dist[i]/norm);
            kernel_struct[i+1].index = i+1;
            kernel_struct[i+1].value = kernel[i];        
    //        printf("%.8f\n", kernel[i]);
        }
        kernel_struct[num_training+1].index = -1;

        // Classify
        max_prob = -1000;
        max_prob_ind = 0;
        for (i=0; i<num_classes; i++){
            svm_get_labels(models[i], label);
            cind = (label[0]==1? 0 : 1);
            svm_predict_probability(models[i], kernel_struct, p);
    //        cout << p[0] << ' ' << p[1] << endl;
            probs[i] = p[cind];
            if (probs[i] > max_prob) {
                max_prob = probs[i];
                max_prob_ind = i;
            }
        }
        *result = max_prob_ind;
        *score =  max_prob; 
        free(label);
        free(kernel_struct);
        free(kernel);
        return max_prob_ind;

    }    
    
    int read_file_legacy(const char * file, double ** M, int *num_lines, int *dim) {
    
        // Declarations
        int line_counter, element_counter;
        double num;

        // Open file
        ifstream fs(file, iostream::in);
        if (!fs.is_open()) {
            cout << "error opening file: " << file << endl;
            return 1;
        }

        // Find out dimensionality
        string line;
        if (getline(fs, line)) {
            istringstream strm(line);
            *dim = 0;
            while ( strm >> num )
                (*dim)++;
            cout << "dimensionality: " << *dim << std::endl;
        }

        // Count lines
        *num_lines = 1; // 1st line already parsed
        while (std::getline(fs, line))
            ++(*num_lines);
        cout << "number of elements: " << *num_lines << std::endl;

        // Reset file
        fs.clear();
        fs.seekg(0, ios::beg);

        // Allocate
    //    M = (double **) malloc(sizeof(double *)*((*num_lines)*(*dim)));
        *M = (double *) malloc(sizeof(double)*((*num_lines)*(*dim)));

        //Read file
        //vector< vector<double> > M;
        line_counter = 0;
        while (getline(fs, line))
        {
            istringstream strm(line);
    //        M[line_counter] = (double *) malloc(sizeof(double)*((*dim)+1));
            element_counter = 0;
            while ( strm >> num )
    //           M[line_counter][element_counter++] = num;
                (*M)[line_counter*(*dim)+(element_counter++)] = num;
            line_counter++;
        }
        fs.close();
        return 0;    


    }
    
    int read_features_stream(std::ostringstream * features_stream, int *num_lines, double ** M, int *dim) {
 
        // Declarations
        int line_counter, element_counter;
        double num;

        // Open file
//        ifstream fs(file, iostream::in);
//        if (!fs.is_open()) {
//            cout << "error opening file: " << file << endl;
//            return 1;
//        }
//        ostringstream s;
//        s << fs.rdbuf();    
//        fs.close();
        std::istringstream is((*features_stream).str());
        

        // Find out dimensionality
        string line;
        if (std::getline(is, line)) {
            istringstream strm(line);
            *dim = 0;
            while ( strm >> num )
                (*dim)++;
//            cout << "dimensionality: " << *dim << std::endl;
        }

        // Count lines
        *num_lines = 1; // 1st line already parsed
        while (std::getline(is, line))
            ++(*num_lines);
//        cout << "number of elements: " << *num_lines << std::endl;

        // Reset file
        is.clear();
        is.seekg(0, ios::beg);

        // Allocate
    //    M = (double **) malloc(sizeof(double *)*((*num_lines)*(*dim)));
        *M = (double *) malloc(sizeof(double)*((*num_lines)*(*dim)));

        //Read file
        //vector< vector<double> > M;
        line_counter = 0;
        while (getline(is, line))
        {
            istringstream strm(line);
    //        M[line_counter] = (double *) malloc(sizeof(double)*((*dim)+1));
            element_counter = 0;
            while ( strm >> num )
    //           M[line_counter][element_counter++] = num;
                (*M)[line_counter*(*dim)+(element_counter++)] = num;
            line_counter++;
        }

        return 0;    


    }
    
    int recognize_legacy(double *features, int num_features){
        BagOfWords(K, D, num_features, features, &bow);
        ChiSquared(K, num_encoded, encoded_features, bow, &dist);
        classify(dist, models, *norm, num_encoded, num_classes, &result, &score, probs);        
        free(bow);
        free(dist);
        return result;
    }
    
    int recognize(std::ostringstream *features_stream){
        int num_features, dim;
        double * features;
        read_features_stream(features_stream, &num_features, &features, &dim);
        BagOfWords(K, D, num_features, features, &bow);
        ChiSquared(K, num_encoded, encoded_features, bow, &dist);
        classify(dist, models, *norm, num_encoded, num_classes, &result, &score, probs); 
        free(bow);
        free(dist);
        return result;
    }
    
    
    
};

//int main(int argc, char** argv) {
//    std::cout << "Hello world!" <<std:: endl;
//    const char * codebook_file = "/home/nick/Dropbox/rtg/data/codebook_MBH";
//    const char * encoded_features_file = "/home/nick/Dropbox/rtg/data/encoded_features_MBH";
//    const char * features_file = "/home/nick/Dropbox/rtg/data/nikos_WantStandUp_1";
//    const char * norm_file = "/home/nick/Dropbox/rtg/data/norm_MBH";
//    std::string model_files = "/home/nick/Dropbox/rtg/data/model";
//    const int num_classes = 12;
//    
// 
//    double * features;
//    int num_features, dim_features;
//    
//    recognizer r(codebook_file, encoded_features_file, norm_file, model_files, num_classes);
//    
//    read_file_legacy(features_file, &features, &num_features, &dim_features);
//    r.recognize(features, num_features);
//    free(features);
//    
//    features_file = "/home/nick/Dropbox/rtg/data/nikos_ComeHere_2";
//    read_file_legacy(features_file, &features, &num_features, &dim_features);
//    r.recognize(features, num_features);
//        
//    return 0;
//}

#endif /* RECOGNIZER_H */


/**
 * @file log_data.hpp
 * @brief      This file contains the functions to log data to a CSV file.
 *      The data is saved in the ./data/ directory, relative to the executable file.
 *      The data is saved in the following format:
 *      - Each line represents a new data point
 *      - Each value is separated by a comma
 *      - The first line is the header (comma separated)
 *      - The header is initialized when the file is created
 *      - The number of columns is initialized when the file is created
*/

#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdarg>
#include <unordered_map>

// The data is saved in the ./data/ directory, relative to the executable file
static const std::string data_dir = "./data/";

// Store the file streams in a map 
static std::unordered_map<std::string, std::ofstream*> files;

/**
 * @brief      Initializes a CSV file (either creates or overwrites)
 * @param[in]  filename The file name (will be stored in the ./data/ directory)
 * @param[in]  header   The header (comma separated, make sure to add a comma after the last column name!)
 * @return     int: number of columns in the CSV file, -1 if the file could not be opened
*/
int init_csv(std::string filename, std::string header){

    // Open the CSV file for writing (overwrite if it already exists)
    files[filename] = new std::ofstream(data_dir + filename, std::ios::trunc);
    
    // Check the file is opened correctly
    if (!files[filename]->is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return -1;
    }
    else
        std::cout << "Opened file: " << filename << std::endl;

    // Write the header to the CSV file
    *files[filename] << header << std::endl;

    // Close the file and reopen it in append mode
    files[filename]->close();
    files[filename]->open(data_dir + filename, std::ios::app);

    // Count the number of commas in the header
    int n_commas = 0;
    for(char c: header){
        if(c ==',') n_commas++;
    }

    return n_commas;
}

/**
 * @brief      Write a new line to a CSV file. Note this function
 *             uses Variadic Arguments and can therefore take
 *             any number of values (double) as input.
 * @param[in]  filename The file name
 * @param[in]  n        The number of values to log
 * @param[in]  ...      The values to log (all doubles)
 * @return     bool: true if the data was successfully logged, 
 *             false otherwise
*/
bool log_csv(std::string filename, int n, ...){

    // Check the file is opened correctly
    if(files.find(filename) == files.end() || !files[filename]->is_open()){
        std::cerr << "Error logging to file: " << filename << std::endl;
        return false;
    }

    // Initialize the variadic argument list
    va_list args;
    va_start(args, n);

    // Loop through the variadic arguments and write them to the CSV file
    float value;
    for(int i=0; i<n; i++){
        value = va_arg(args, double);
        *files[filename] << value << ", ";
    }
    *files[filename] << std::endl;

    // End the variadic argument list
    va_end(args);

    return true;
}

/**
 * @brief      Close a previously opened CSV file
 * @param[in]  filename The file name
 * @return     bool: true if the file was successfully closed,
*/
bool close_csv(std::string filename){

    // Check the file was previously opened
    if(files.find(filename) == files.end() || !files[filename]->is_open()){
        std::cerr << "Error closing file: " << filename << std::endl;
        return false;
    }
    else
        std::cout << "Closing file: " << filename << std::endl;

    files[filename]->close();
    delete files[filename];
    files.erase(filename);

    return true;
}

/**
 * @brief      Close all previously opened CSV files
*/
void close_csv(){
    
    for (auto it : files){
        it.second->close();
        delete it.second;
    }
    
    files.clear();

    std::cout << "Closed all files" << std::endl;
}
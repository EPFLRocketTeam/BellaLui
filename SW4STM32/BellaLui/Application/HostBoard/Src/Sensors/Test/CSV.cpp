/*
 * CSV.cpp
 *
 *  Created on: 25 Nov 2020
 *      Author: Arion
 */


#include "CSV.h"
#include <stdexcept>

std::vector<std::vector<float>> read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::vector<float>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    float val;

    // Read the column names
    if(myFile.good()) {
        // Ignore the first line in the file
        std::getline(myFile, line);

        std::stringstream ss(line);

		// Extract each column name
		while(std::getline(ss, colname, ',')){

			// Initialize and add <colname, int vector> pairs to result
			result.push_back(std::vector<float> {});
		}
    }

    // Read data, line by line
    while(std::getline(myFile, line)) {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        int colIdx = 0;

        if(ss.peek() == ',') ss.ignore();

        // Extract each integer
        while(ss >> val){

            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).push_back(val);

            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();

            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

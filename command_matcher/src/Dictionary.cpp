class Dictionary {
   public:
    /* Function that returns the size of the dictionary.
     * 
     * INPUT: none
     * OUTPUT: none
     */
      int size() {
         return list.size();
      }

    /* Function that adds an entry to the list vector.
     * 
     * INPUT: string filename: filename with dictionary file
     * OUTPUT: none
     */
      void add_entry(string str) {
      list.push_back(str);
    }

    /* Function that returns the index of the best match for str in list. 
     * If no good  match is found, -1 is returned.
     * 
     * INPUT: string str: sample string
     * OUTPUT: int: index of best match, -1 if no good match found
    */
      int findBestMatch(string str) {
         int rating;
         int bestRating = 0;
         int bestRated;
         bool rightOrder;
         size_t lastPos;
         
         for(size_t i = 0; i < list.size(); i++) { // for all entries in dictionary (an entry may contain multiple words)
            rating = 0;
            rightOrder = true;
            lastPos = -1;

        // separate strings into words
            vector<string> dictWords = stringToWords(list[i]);
            int dictWordsOrigSize = dictWords.size();

            vector<string> strWords = stringToWords(str);
            int strWordsOrigSize = strWords.size();

            // compare each word from dict entry to each word from str
            for(size_t j = 0; j < dictWords.size(); j++) { // for all words in current dictionary entry
               for(size_t k = 0; k < strWords.size(); k++) { // for all words in currently recognized string
                  if(strWords[k].compare(dictWords[j]) == 0) { // dictWords contains a word from str
                     if(lastPos > j) rightOrder = false; // wrong word order detected, rating-- would be more aggressive
                     lastPos = j;
                     rating++;
              // remove words
                     strWords.erase(strWords.begin() + k );
                     k--;
                     dictWords.erase(dictWords.begin() + j);
                     j--;
                     break;
                  }
               }
            }
        // best match only matches half or less of the words
            if(rating < dictWordsOrigSize/2) {
              rating = 0;
            }
        
        // bonuses for correct length or correct order
            if(rating > 0) {
               if(rating == dictWordsOrigSize) rating++; // bonus if all words are equal, but order may be different
               if(rightOrder && strWordsOrigSize > 1) rating++; // bonus for correct word order of non-trivial str
            }
            
            if(rating > bestRating) { // update rating
          bestRating = rating;
          bestRated = i;
            }
         }
      // no match found
         if(bestRating == 0) bestRated = -1;
         
         return bestRated;
      }
            
   private:
      vector<string> list;
      
    /* Function that splits a string str up at space characters and
     * returns the segments in a vector<string>.
     * 
     * INPUT: string str: string to be split
     * OUTPUT: vector<string>: vector of string segments
     */
      vector<string> stringToWords(string str) {
         vector<string> words;
         size_t begin = 0;
         size_t end = str.find_first_of(' ', begin);
         while(end != string::npos) {
            words.push_back(str.substr(begin, end-begin));
            begin = end + 1; // skip space
        end = str.find_first_of(' ', begin);
      }
      words.push_back(str.substr(begin, str.length() - begin));
      return words;
    }
};

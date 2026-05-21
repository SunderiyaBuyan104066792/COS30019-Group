Please follow these steps to set up everything

1. Preparing the datasets:
    From to PartB/data directory on your terminal:
        1.1 Upload the "Scats Data October 2006.xls" in the data folder
        1.2 Run python process_df.py
        Will output you the SCATS_Data containing all required train/test sets 

2. Getting all the trained models:
    The models were traind by "train.py" using the SCATS_Data and "model.py"
    2.1 Download the zipped file "trained_models" and save and unzip it within the PartB directory. 

3. Adding the custom model: 
    1. If you need a different preprocessing, training etc create a separate file for it. We can combine it later.

4. Testing the models will take few minutes:
    4.1 Go to the PartB directory
    4.2 Currenty you can test individual direction for sites by defining the site number and direction on the "main()" It will give you the metrics and the graph. 
    4.3 Additionally, I have added evaluate all that gives the average metrics for both models. Just close the graph window and wait for a minute for this. Ignore the warnings. 

    The testing needs to be worked further. 

5. Graph/Edges/Time calculations
    5.1 Go to the graph folder
    5.2 Upload the "SCATSSiteListingSpreadsheet_VicRoads.xls" like this: "site_road_data/SCATSSiteListingSpreadsheet_VicRoads.xls
    5.3 Run "python parse_road_data.py" to get the data needed for graph
    5.4 Run "python parse_site_types.py" to check whether the SCATS are intersections(ALL were INT). 



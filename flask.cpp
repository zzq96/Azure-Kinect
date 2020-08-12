#include"flask.h"
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

string send_char_arr(char * memblock, long size){
    CURL *curl;
    CURLcode res;

    string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, "http://49.52.10.229:5000/predict");

        // disable Expect:
        struct curl_slist *chunk = NULL;
        chunk = curl_slist_append(chunk, "Content-Type: opencv/image"); // use strange type to prevent server to parse content
        chunk = curl_slist_append(chunk, "Expect:");
        res = curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);

        // data
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, memblock);
        // data length
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, size);

        // response handler
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        /* Perform the request, res will get the return code */ 
        res = curl_easy_perform(curl);

        /* Check for errors */ 
        if(res != CURLE_OK){
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
            curl_easy_strerror(res));
        }

        /* always cleanup */ 
        curl_easy_cleanup(curl);
    }

    //delete[] memblock; // memblock should be freed from called here.
    return readBuffer;
}

string send_mat(cv::Mat mat){
    vector<uchar> buf;
    imencode(".png", mat, buf);
    char * memblock = reinterpret_cast<char*>(buf.data());
    long size = buf.size();

    return send_char_arr(memblock, size);
}

void getMask(cv::Mat& mat, vector<vector<cv::Point2d>> & points)
{
	string resp = send_mat(mat);
	Document doc;
	doc.Parse(resp.c_str());
	bool success = doc["success"].GetBool();
	if (doc.HasMember("success") && doc["success"].IsBool())
	{
		cout << "get mask "<<success << endl;
	}
	//5.1 整型数组类型
	int id, row, col;
    points.clear();
	if(doc.HasMember("instances_id") && doc["instances_id"].IsArray()\
		&&doc.HasMember("instances_row") && doc["instances_row"].IsArray()
		&&doc.HasMember("instances_col") && doc["instances_col"].IsArray())
	{
		//5.1.1 将字段转换成为rapidjson::Value类型
		const rapidjson::Value& ids = doc["instances_id"];
		const rapidjson::Value& rows = doc["instances_row"];
		const rapidjson::Value& cols = doc["instances_col"];
		//5.1.2 获取数组长度
		size_t len = ids.Size();
		//5.1.3 根据下标遍历，注意将元素转换为相应类型，即需要调用GetInt()
		for(size_t i = 0; i < len; i++)
		{
			id = ids[i].GetInt();
			row = rows[i].GetInt();
			col = cols[i].GetInt();
			if (id + 1 > points.size())
				points.push_back(vector<cv::Point2d>());
			points[id].push_back(cv::Point2d(col, row));
		}
	}
	//cout << "issuccess: " << issuccess  << endl;
	//GenericArray  arr = d["instances"].GetArray();
}

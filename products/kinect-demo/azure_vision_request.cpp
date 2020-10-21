#include <stdio.h>
#include "azure_vision_request.h"
#include <cpprest/filestream.h>
#include <iostream>
#include <string>

// Taken from https://github.com/deercoder/cpprestsdk-example

/**
https://westus.api.cognitive.microsoft.com/vision/v1.0/analyze[?visualFeatures][&details][&language]

visualFeatures can be: Categories, Tags, Faces, ImageType, Color, Adult
details can be: Celebrities
language can be: en, zh

Only one value can be set at a time.

Reference:
1) Console debug tools: https://westus.dev.cognitive.microsoft.com/docs/services/56f91f2d778daf23d8ec6739/operations/56f91f2e778daf14a499e1fa/console
2) API interface: https://westus.dev.cognitive.microsoft.com/docs/services/56f91f2d778daf23d8ec6739/operations/56f91f2e778daf14a499e1fa
*/
std::string vision_base_host ="";


const std::string const_vision_host = "";
std::string feature_options[5] = {"Objects", "Brands", "Categories", "Description", "Faces"};

AzureVisionRequest::AzureVisionRequest(int flag)
{
    char *tmp = getenv("AZURE_VISION_ENDPOINT");
    if (tmp != NULL)
        vision_base_host = getenv("AZURE_VISION_ENDPOINT");
    this->setRequestType(flag);
    // this->client = new http_client(conversions::to_string_t(vision_base_host));
    this->client = new http_client(vision_base_host);
    this->request = new http_request(methods::POST);
    this->send_raw_image = true;
}

void AzureVisionRequest::setRequestType(int flag)
{
    if (flag >= 0 && flag < 4)
    {
        vision_base_host = vision_base_host + feature_options[flag];
    }
    else
    {
        std::cout << "Wrong flag, exceeding the range!" << std::endl;
    }
}

void AzureVisionRequest::ConstructRequestHeader(const char *subscription_key)
{
    this->request->headers().add("Content-Type", "application/octet-stream");
    this->request->headers().add("Ocp-Apim-Subscription-Key", subscription_key);
}

void AzureVisionRequest::ConstructRequestBody(std::vector<uchar> buff)
{

    concurrency::streams::istream input_stream = concurrency::streams::bytestream::open_istream(buff);
    // concurrency::streams::istream input_stream = concurrency::streams::file_stream<uint8_t>::open_istream(conversions::to_string_t(image_path)).get();
    /*
		Must specify the length of the stream, otherwise it shows ERROR 400: image size too small or too large
		even though their API supports non-length parameters, check here:
		https://social.msdn.microsoft.com/Forums/en-US/6fb47e0d-fc9e-44f0-af3d-66887e10a72c/face-api-error-invalidimagesize-image-size-is-too-small-or-too-big-for-each-request?forum=mlapi
		*/
    this->request->set_body(input_stream, buff.size(), "application/octet-stream");
}

http_client *AzureVisionRequest::getClient()
{
    return this->client;
}

http_request *AzureVisionRequest::getRequest()
{
    return this->request;
}

int AzureVisionRequest::SendAnalysisRequest(const char *key, std::vector<uchar> buff)
{
    auto fileStream = std::make_shared<ostream>();

    // Open stream to output file.
    pplx::task<void> requestTask = fstream::open_ostream(_XPLATSTR("results2.html")).then([=](ostream outFile) {
                                                                                        *fileStream = outFile;
                                                                                        this->ConstructRequestHeader(key);
                                                                                        this->ConstructRequestBody(buff);
                                                                                        return this->getClient()->request(*this->getRequest());
                                                                                    })
                                       // Handle response headers arriving.
                                       .then([=](http_response response) {
                                        //    printf("Received response status code:%u\n", response.status_code());
                                           utility::string_t tt = response.extract_string().get();
                                           std::string str(conversions::to_utf8string(tt));
                                        //    std::cout << "JSON string is: " << str << std::endl;

                                           json::value obj = json::value::parse(conversions::to_string_t(str));

                                           if (obj.has_field("objects"))
                                           {
                                               json::array obs = obj.at("objects").as_array();
                                               for (size_t i = 0; i < obs.size(); i++)
                                               {
                                                   json::value jv = obs[i];
                                                   std::string object_name = conversions::to_utf8string(jv.at("object").as_string());
                                                   double confidence = jv.at("confidence").as_number().to_double();
                                                   this->object_map.insert(std::pair<std::string, double>(object_name, confidence));
                                               }
                                           }

                                           /* if request type is the `tags`, then parse the tags information and its score */

                                           /* categories contains multiple elements, if there are, otherwise there is only one or empty,
		   so we should consider for all cases, not just one or two, use general iterations to represent
		 */
                                           if (obj.has_field("categories"))
                                           {
                                               json::array cats = obj.at("categories").as_array();
                                               std::string concat_cat_str = "";
                                               std::string concat_cat_score = "";
                                               for (size_t i = 0; i < cats.size(); i++)
                                               {
                                                   json::value jv = cats[i];
                                                   concat_cat_str += conversions::to_utf8string(jv.at("name").as_string());
                                                   concat_cat_score += std::to_string(jv.at("score").as_number().to_double());
                                                   if (i != cats.size() - 1)
                                                   {
                                                       concat_cat_str += ",";
                                                       concat_cat_score += ",";
                                                   }
                                               }
                                               this->setCategoryNames(concat_cat_str);
                                               this->setCategoryScores(concat_cat_score);
                                           }

                                           /* if request type is the `tags`, then parse the tags information and its score */
                                           if (obj.has_field("tags"))
                                           {
                                               json::array tags = obj.at("tags").as_array();
                                               std::string concat_tags = "";
                                               std::string concat_conf = "";
                                               for (size_t i = 0; i < tags.size(); i++)
                                               {
                                                   json::value jv = tags[i];
                                                   concat_tags += conversions::to_utf8string(jv.at("name").as_string());
                                                   concat_conf += std::to_string(jv.at("confidence").as_number().to_double());
                                                   if (i != tags.size() - 1)
                                                   {
                                                       concat_tags += ",";
                                                       concat_conf += ",";
                                                   }
                                                   //std::cout << conversions::to_utf8string(jv.at("name").as_string()) << std::endl;
                                                   //std::cout << jv.at("confidence").as_number().to_double() << std::endl;
                                               }
                                               this->setImageTagsInfo(concat_tags);
                                               this->setImageTagsConfidenceInfo(concat_conf);
                                           }

                                           /* if request type is the `description`, then parse the caption and its tags. (this tags have more than above tag request) */
                                           if (obj.has_field("description"))
                                           {
                                               json::object jb = obj.at("description").as_object();
                                               json::array des_tags = jb.at("tags").as_array();
                                               json::array des_catpions = jb.at("captions").as_array();

                                               std::string caption_tags = "";
                                               std::string caption = "";
                                               std::string caption_conf = "";

                                               for (size_t i = 0; i < des_tags.size(); i++)
                                               {
                                                   json::value jv = des_tags[i];
                                                   //std::cout << conversions::to_utf8string(jv.as_string()) << std::endl;
                                                   caption_tags += conversions::to_utf8string(jv.as_string());

                                                   if (i != des_tags.size() - 1)
                                                   {
                                                       caption_tags += ",";
                                                   }
                                               }

                                               for (size_t j = 0; j < des_catpions.size(); j++)
                                               {
                                                   json::value jv = des_catpions[j];
                                                   //std::cout << conversions::to_utf8string(jv.at("text").as_string()) << std::endl;
                                                   //std::cout << jv.at("confidence").as_number().to_double() << std::endl;
                                                   caption += conversions::to_utf8string(jv.at("text").as_string());
                                                   caption_conf += std::to_string(jv.at("confidence").as_number().to_double());

                                                   if (j != des_catpions.size() - 1)
                                                   {
                                                       caption += ",";
                                                       caption_conf += ",";
                                                   }
                                               }

                                               this->setImageCaptionsInfo(caption_tags, caption, caption_conf);
                                           }

                                           if (obj.has_field("metadata"))
                                           {
                                               json::object jb = obj.at("metadata").as_object();
                                               //std::cout << jb.at("width").as_number().to_double() << std::endl;
                                               //std::cout << jb.at("height").as_number().to_double() << std::endl;
                                               //std::cout << conversions::to_utf8string(jb.at("format").as_string()) << std::endl;

                                               this->setImageWidth(jb.at("width").as_number().to_double());
                                               this->setImageHeight(jb.at("height").as_number().to_double());
                                               this->setImageFormat(conversions::to_utf8string(jb.at("format").as_string()));
                                           }

                                           // Write response body into the file.
                                           return response.body().read_to_end(fileStream->streambuf());
                                       })

                                       // Close the file stream.
                                       .then([=](size_t) {
                                           return fileStream->close();
                                       });

    // Wait for all the outstanding I/O to complete and handle any exceptions
    try
    {
        requestTask.wait();
    }
    catch (const std::exception &e)
    {
        printf("Error exception:%s\n", e.what());
    }
    return 0;
}

void AzureVisionRequest::setImageCaptionsInfo(std::string t, std::string c, std::string cf)
{
    this->image_captions_tags = t;
    this->image_captions = c;
    this->image_caption_confs = cf;
}

std::string AzureVisionRequest::getImageCaptionInfo()
{
    return this->image_captions;
}

std::string AzureVisionRequest::getImageCaptionConfidenceInfo()
{
    return this->image_caption_confs;
}

std::string AzureVisionRequest::getImageCaptionTagsInfo()
{
    return this->image_captions_tags;
}

void AzureVisionRequest::setImageTagsInfo(std::string a)
{
    this->image_tags = a;
}

std::string AzureVisionRequest::getImageTags()
{
    return this->image_tags;
}

void AzureVisionRequest::setImageTagsConfidenceInfo(std::string a)
{
    this->image_tags_confidence = a;
}

std::string AzureVisionRequest::getImageTagsConfidenceInfo()
{
    return this->image_tags_confidence;
}

void AzureVisionRequest::setImageWidth(double a)
{
    this->img_width = a;
}

void AzureVisionRequest::setImageHeight(double b)
{
    this->img_height = b;
}

void AzureVisionRequest::setImageFormat(std::string s)
{
    this->img_format = s;
}

void AzureVisionRequest::setCategoryNames(std::string s)
{
    this->cat_names = s;
}

void AzureVisionRequest::setCategoryScores(std::string s)
{
    this->cat_scores = s;
}

std::string AzureVisionRequest::getCategoryNames()
{
    return this->cat_names;
}

std::string AzureVisionRequest::getCategoryScores()
{
    return this->cat_scores;
}

std::string AzureVisionRequest::getTopObject()
{
    if (this->object_map.size() == 0 ) {
        return "";
    }
    auto x = std::max_element(this->object_map.begin(), this->object_map.end(),
                              [](const std::pair<std::string, double> &p1, const std::pair<std::string, double> &p2) { return p1.second < p2.second; });
    return x->first;
}
AzureVisionRequest::~AzureVisionRequest()
{
    delete client;
    delete request;
}

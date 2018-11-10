#ifndef MONGOOSEHELPER_H
#define MONGOOSEHELPER_H

#include <string>
#include <map>
#include <fstream>


#define STRINGIZE_(x) #x
#define STRINGIZE(x) STRINGIZE_(x)
#define SESSIONTIMEOUT 604800
#define SESSIONIDSIZE 16
#define MULTIUPLOADTIMEOUT 60



namespace Mongoose
{
   typedef enum HttpStatus
   {
	   HttpStatus_SUCCESS_OK=200,
	   HttpStatus_SUCCESS_OK_NO_DATA=204,
	   HttpStatus_CLIENT_ERROR_BAD_REQUEST=400,
	   HttpStatus_CLIENT_ERROR_UNAUTHORIZED=401,
	   HttpStatus_CLIENT_ERROR_NOT_FOUND=404 ,
	   HttpStatus_SERVER_ERROR_INTERNAL=500 ,
	   HttpStatus_SERVICE_OVERLOADED=502
   }tHttpStatus;

   class CHttpConnWrapper
   {
      public:
         class CHttpConnWrapperImpl;

         CHttpConnWrapper(const CHttpConnWrapper& httpConn);
         explicit CHttpConnWrapper(CHttpConnWrapperImpl* pImpl);
         ~CHttpConnWrapper();

         const char* GetRequest() const;
         void WriteWebSocketPacket(const char *pMessage);
         size_t ReadFileContentsInBinaryMode(const char *fileName, char **fBuf);
         void HttpSendResponse(tHttpStatus httpStatus,
                               const char *pMessage=NULL, bool attachFile=false,
                                                        const char *fName=NULL);
         void HttpSendCookie(tHttpStatus httpStatus, const char *cookieStr, const char *pBuf);

         void CloseConnection();
         void KeepConnection();
         void SetSessionId(unsigned int sessionId);
         CHttpConnWrapper& operator= (const CHttpConnWrapper& httpConn);



      private:

         const char *GetHttpStatusString(tHttpStatus httpStatusCode);

         CHttpConnWrapperImpl* m_pImpl; ///< pimpl pattern
   };

   class IMongooseCallback
   {
      public:
         IMongooseCallback(){};
         virtual ~IMongooseCallback(){};
         virtual void HttpServicePostCommandCallback(CHttpConnWrapper& httpConn, const std::string& URI, const std::string& URIParameters,
                                                     const char *postBody, size_t postBodyLength, bool& keepOpen) = 0;

   };

   class MongooseHelper
   {
      public:

         explicit MongooseHelper(IMongooseCallback* pInterface);
         ~MongooseHelper();

         bool Initialize(std::string port);

         void CallServiceEvents(double dt);

         IMongooseCallback* GetCallbackInterface() {return callback;}
         std::string m_uploadFolder;
         std::string m_wwwRootFolder;
         void unregisterExpiredSessions(void);
         void unregisterSession(const std::string &webSessionID);
         void BroadcastWebSocketPacket(const char *pBuf);

      private:
         MongooseHelper(MongooseHelper&);
         MongooseHelper& operator= (const MongooseHelper&);

         class MongooseHelperImpl;
         MongooseHelperImpl* m_pImpl;
         IMongooseCallback* callback;

         std::map<std::string, double> m_regSessionID;
         double run_time;
   };
}

#endif

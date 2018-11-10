#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <algorithm>
#include <vector>



#include "MongooseHelper.h"
#include "mongoose.h"

using namespace std;


namespace Mongoose
{
static struct mg_serve_http_opts sHttpServerOptions;
static const struct mg_str HEADER_GET = MG_MK_STR("GET");
static const struct mg_str HEADER_POST = MG_MK_STR("POST");

struct fileWriterData
{
	std::string filename;
	double creationTime;
	FILE *fp;
	unsigned int bytesWritten;
	std::string URI;
};
static MongooseHelper* g_pMongooseInstance = NULL;
struct mg_mgr sMongooseManager;
class CHttpConnWrapper::CHttpConnWrapperImpl
{
	typedef struct mg_connection HttpConn;

public:
	explicit CHttpConnWrapperImpl(HttpConn* pHttpConn);
	~CHttpConnWrapperImpl();
	HttpConn* m_pHttpConn;
};
CHttpConnWrapper::CHttpConnWrapperImpl::CHttpConnWrapperImpl(HttpConn* pHttpConn):
        								 m_pHttpConn(pHttpConn)
{
}
CHttpConnWrapper::CHttpConnWrapperImpl::~CHttpConnWrapperImpl()
{
	m_pHttpConn = NULL;
}
CHttpConnWrapper::CHttpConnWrapper(const CHttpConnWrapper& httpConn) :
        								m_pImpl( NULL )
{
	m_pImpl = new CHttpConnWrapperImpl(httpConn.m_pImpl->m_pHttpConn);
}
CHttpConnWrapper::CHttpConnWrapper(CHttpConnWrapperImpl* pImpl) :
        								m_pImpl( pImpl )
{
}
CHttpConnWrapper::~CHttpConnWrapper()
{
	if (m_pImpl)
	{
		delete m_pImpl;
		m_pImpl = NULL;
	}
}
CHttpConnWrapper& CHttpConnWrapper::operator= (const CHttpConnWrapper& httpConn)
{
	m_pImpl->m_pHttpConn = httpConn.m_pImpl->m_pHttpConn;
	return (*this);
}
void CHttpConnWrapper::SetSessionId(unsigned int sessionId)
{
	m_pImpl->m_pHttpConn->user_data = (void*) sessionId;
}
void CHttpConnWrapper::CloseConnection()
{
	if ( m_pImpl && m_pImpl->m_pHttpConn )
	{
		m_pImpl->m_pHttpConn->flags |= MG_F_SEND_AND_CLOSE;
	}
}
void CHttpConnWrapper::KeepConnection()
{
	if ( m_pImpl && m_pImpl->m_pHttpConn )
	{
		m_pImpl->m_pHttpConn->flags &= ~MG_F_SEND_AND_CLOSE;
	}
}
size_t CHttpConnWrapper::ReadFileContentsInBinaryMode(const char *fileName, char **fBuf)
{   FILE* fStream= (FILE *)NULL;
size_t readCount, fileSize;
char *readBuf;
/*---------------------------------------------------------------------*/

if( (fStream= fopen(fileName, "rb")) == NULL )
{
	return( 0 );
}
fseek(fStream, 0, SEEK_END);
fileSize= ftell(fStream);
rewind(fStream);

if( (readBuf= (char*)calloc(1, sizeof(char)*fileSize)) == (char *)NULL )
{
	fclose(fStream);
	return( 0 );
}
readCount= fread(readBuf, 1, fileSize, fStream);
*fBuf= readBuf;
if( !readCount )
{   fclose(fStream);
free(readBuf);
return( 0 );
}
fclose(fStream);
return( readCount );
}
void CHttpConnWrapper::HttpSendResponse(tHttpStatus httpStatus,
		const char *pBuf, bool attachFile, const char *fName)
{   std::string httpHeader("");
std::ostringstream oStream, tStream;
char *fileBuf= (char *)NULL;

if( attachFile  &&  (fName != (char *)NULL) )
{   std::string fileName(g_pMongooseInstance->m_uploadFolder + fName);
size_t byteCount= ReadFileContentsInBinaryMode(fileName.c_str(), &fileBuf);
if( byteCount == 0 )
{   oStream << std::string("HTTP/1.1  ") << 404 << "File not found\r\n";
oStream << "Content-Length: 0" << "\r\n\r\n";
}
else
{   oStream << std::string("HTTP/1.1  ") << httpStatus;
oStream << " " << GetHttpStatusString(httpStatus) << "\r\n";
oStream << "Content-Type: text/plain; charset=iso-8859-1\r\n";
oStream << "Content-Disposition: attachment; filename=" << fName << "\r\n";
oStream << "Content-Transfer-Encoding: binary" << "\r\n";
oStream << "Content-Length: " << byteCount << "\r\n\r\n";
}
httpHeader= oStream.str();

mg_printf(m_pImpl->m_pHttpConn, "%s", httpHeader.c_str());
mg_send(m_pImpl->m_pHttpConn, fileBuf, byteCount);
if( byteCount != 0 )  free(fileBuf);
}
else
{   if( pBuf )
{   int bufLen= strlen(pBuf);
mg_printf( m_pImpl->m_pHttpConn,
		"HTTP/1.1 %d %s\r\n"
		"Content-Type: text/plain\r\n"
		"Content-Length: %d\r\n\r\n%s",
		httpStatus,
		GetHttpStatusString(httpStatus),
		bufLen,
		pBuf
);
}
else
{   mg_printf( m_pImpl->m_pHttpConn,
		"HTTP/1.1 %d %s\r\n"
		"Content-Type: text/plain\r\n"
		"Content-Length: 0\r\n\r\n",
		httpStatus,
		GetHttpStatusString(httpStatus)
);
}
}
}
void CHttpConnWrapper::HttpSendCookie(tHttpStatus httpStatus, const char *cookieStr, const char *pBuf)
{
	if(pBuf)
	{
		int bufLen = strlen(pBuf);
		mg_printf(m_pImpl->m_pHttpConn, "HTTP/1.1 %d %s\r\n"
				"Content-Type: text/plain\r\n"
				"Content-Length: %d\r\n"
				"%s\r\n\r\n%s",
				httpStatus,
				GetHttpStatusString(httpStatus),
				bufLen,
				cookieStr,
				pBuf
		);
	}
	else
	{
		mg_printf(m_pImpl->m_pHttpConn, "HTTP/1.1 %d %s\r\n"
				"%s\r\n\r\n",
				httpStatus,
				GetHttpStatusString(httpStatus),
				cookieStr
		);
	}
}
void CHttpConnWrapper::WriteWebSocketPacket(const char* pBuf)
{
	mg_send_websocket_frame(m_pImpl->m_pHttpConn, WEBSOCKET_OP_TEXT, pBuf, strlen(pBuf));
}

class MongooseHelper::MongooseHelperImpl
{
public:
	MongooseHelperImpl();
	~MongooseHelperImpl();

};
MongooseHelper::MongooseHelperImpl::MongooseHelperImpl()
{
}
MongooseHelper::MongooseHelperImpl::~MongooseHelperImpl()
{
}
MongooseHelper::MongooseHelper(IMongooseCallback* pInterface) :
        								m_pImpl( new MongooseHelperImpl() ),
										callback(pInterface)
{
	g_pMongooseInstance = this;
}
MongooseHelper::~MongooseHelper()
{
	if ( m_pImpl != NULL )
	{
		delete m_pImpl;
		m_pImpl = NULL;
	}

	g_pMongooseInstance = NULL;
}
void MongooseHelper::BroadcastWebSocketPacket(const char *pBuf)
{
	if(sMongooseManager.active_connections)
	{
		struct mg_connection *itConnection =
				sMongooseManager.active_connections;
		while(itConnection != NULL)
		{
			if(itConnection->flags & MG_F_IS_WEBSOCKET)
			{
				mg_send_websocket_frame(itConnection
						, WEBSOCKET_OP_TEXT
						, pBuf, strlen(pBuf));

			}
			itConnection = itConnection->next;
		}
	}
}
static bool isMgStringEqual(const struct mg_str *pMgString1, const struct mg_str *pMgString2)
{
	return ((pMgString1->len == pMgString2->len) &&( memcmp(pMgString1->p, pMgString2->p, pMgString2->len) == 0));
}
std::string getSessionID(struct http_message *pHttpReq)
{
	int i;
	std::string webSessionID("");
	std::string headerName("");
	std::size_t pos;

	for(i = 0; pHttpReq->header_names[i].len > 0; i++)
	{
		headerName.clear();
		if((pHttpReq->header_names[i].len > 0) && (pHttpReq->header_names[i].p != NULL))
		{
			headerName = std::string(pHttpReq->header_names[i].p, pHttpReq->header_names[i].len);
			if(!headerName.compare("Cookie"))
			{
				webSessionID = std::string(pHttpReq->header_values[i].p, pHttpReq->header_values[i].len);
			}
		}
	}

	pos = webSessionID.find("sessionID=");
	if(pos != std::string::npos)
	{
		pos += strlen("sessionID=");
		webSessionID = webSessionID.substr(pos, std::string::npos);
		pos = webSessionID.find(";");
		if(pos != std::string::npos)
		{
			webSessionID = webSessionID.substr(0, pos);
		}
	}
	else
	{
		webSessionID.clear();
	}

	return webSessionID;
}

void MongooseHelper::unregisterSession(const std::string &webSessionID)
{
	std::map<std::string, double>::iterator itr;

	itr = m_regSessionID.find(webSessionID);
	if(itr != m_regSessionID.end())
	{
		m_regSessionID.erase(itr++);
	}
}
void MongooseHelper::unregisterExpiredSessions(void)
{
	std::map<std::string, double>::iterator itr;
	for(itr = m_regSessionID.begin(); itr != m_regSessionID.end(); )
	{
		if((run_time - itr->second) > SESSIONTIMEOUT)
		{
			m_regSessionID.erase(itr++);
		}
		else
		{
			++itr;
		}
	}
}
void ReceptionHandler(struct mg_connection *pConnection, int event, void *ev_data)
{
	static std::string URI;
	static std::string Query;

	struct fileWriterData** multipartData = (struct fileWriterData **) &pConnection->user_data;
	struct mg_http_multipart_part *mp = (struct mg_http_multipart_part *) ev_data;

	std::string filename;
	std::ifstream source;
	std::ofstream dest;
	struct http_message *pHttpReq = (struct http_message *) ev_data;
	bool keepOpen = false;
	CHttpConnWrapper httpConn(new CHttpConnWrapper::CHttpConnWrapperImpl(pConnection));

	std::size_t curIndex;
	std::string webSessionID("");
	std::string cookieStr("");
	bool isAuth = false;
	if(event != MG_EV_POLL)
	{

	}

	switch (event)
	{
	case MG_EV_HTTP_REQUEST:

		if(pHttpReq == NULL || pHttpReq->method.p == NULL)
		{
			return;
		}
		webSessionID = getSessionID(pHttpReq);
		if( isMgStringEqual(&pHttpReq->method, &HEADER_POST) == true )
		{
			URI.clear();
			Query.clear();
			if( (pHttpReq->uri.len > 0)  &&  (pHttpReq->uri.p != (char *) NULL) )
			{
				URI= std::string(pHttpReq->uri.p);
				URI.erase(pHttpReq->uri.len, std::string::npos);
			}
			if( (pHttpReq->query_string.len > 0)  &&  (pHttpReq->query_string.p != (char *) NULL) )
			{
				Query= std::string(pHttpReq->query_string.p);
				curIndex = 0;
				Query.erase(pHttpReq->query_string.len, std::string::npos);
				while( (curIndex= Query.find("%20", curIndex)) !=  std::string::npos )
				{   Query.replace(curIndex, 3, " ");
				}
				Query.erase(Query.length(), std::string::npos);
			}
			const char* end_point = std::find(pHttpReq->body.p, pHttpReq->body.p+pHttpReq->body.len, 0);
			std::string body(pHttpReq->body.p, end_point);
			g_pMongooseInstance->GetCallbackInterface()->HttpServicePostCommandCallback(httpConn, URI,
									Query, body.c_str(), body.size(), keepOpen);

			if(keepOpen)
			{
				httpConn.KeepConnection();
			}
			else
			{
				httpConn.CloseConnection();
			}
		}
		else if ( isMgStringEqual(&pHttpReq->method, &HEADER_GET) == true)
		{
			mg_serve_http(pConnection, pHttpReq, sHttpServerOptions);
			pConnection->flags |= MG_F_SEND_AND_CLOSE;
		}
		break;
	default:
		break;
	}
}
bool MongooseHelper::Initialize(std::string port)
{
	bool status = false;
	struct mg_connection *nc;

	mg_mgr_init(&sMongooseManager, NULL);
	nc = mg_bind( &sMongooseManager,
			port.c_str(),
			ReceptionHandler);

	if(nc != NULL)
	{
		mg_set_protocol_http_websocket(nc);

		sHttpServerOptions.document_root = g_pMongooseInstance->m_wwwRootFolder.c_str();
		sHttpServerOptions.enable_directory_listing = "yes";
		status = true;
	}
	return status;
}


const char *CHttpConnWrapper::GetHttpStatusString(tHttpStatus httpStatus)
{
	switch(httpStatus)
	{
	case HttpStatus_SUCCESS_OK:                 return "OK"; break;
	case HttpStatus_SUCCESS_OK_NO_DATA:         return "NO DATA"; break;
	case HttpStatus_CLIENT_ERROR_BAD_REQUEST:   return "BAD REQUEST"; break;
	case HttpStatus_CLIENT_ERROR_UNAUTHORIZED:  return "UNAUTHORIZED"; break;
	case HttpStatus_CLIENT_ERROR_NOT_FOUND:     return "NOT FOUND"; break;
	case HttpStatus_SERVICE_OVERLOADED:         return "SERVICE OVERLOADED"; break;
	default:
		break;
	}
	return "";
}
void MongooseHelper::CallServiceEvents(double dt)
{
	run_time += dt;
	(void) mg_mgr_poll(&sMongooseManager, 0);
}
}


// #include <log/log.h>
#include "MessParser.h"

MessParser::MessParser()
{
    m_messLineSep = GNSS_NMEA_LINE_SEP;
    m_messElementSep = GNSS_NMEA_ELEMENT_SEP;
 
    reset();
}

MessParser::~MessParser()
{
    reset();
}

// 全部清空和重新分配空间
void MessParser::reset()
{
   m_messLines.clear();
   m_messElementList.clear();
}
// 解析mess
bool MessParser::parse(const std::string &mess)
{
    reset();
    // int gpCount = 0;
    if (0 == mess.size()) // 如果报文没东西
    {
        return 0;
    }
    (void)split(mess, m_messLineSep, m_messLines); // mess根据messLineSep拆分，存入messLines中
    std::vector<std::string>::iterator vsit;

    std::string line;

    headerList.clear();
    // 遍历拆分的字符串段
    for (vsit = m_messLines.begin(); vsit != m_messLines.end(); vsit++)
    {
        line = *vsit; // 这句报文
        if (line.size() <= 6)
        {
            //$GPxxx
            continue;
        }

                               
        string MessageHeader;
        MessageHeader = line.substr(0, 6); // 切开

        // 如果报文头部对了，那就拆分解析它
        if (startsWith(line, "xxxx"))
        { // GGA
            (void)split(line, m_messElementSep, m_messElementList);
            headerList.push_back(MessageHeader);
        }
        else
        {
            printf_color(PrintColor::Yellow, "unkown line:%s\n", line.c_str());
        }
    }

 
    return true;
}
//
int MessParser::getNmeaLines(std::vector<std::string> &lines)
{
    std::vector<std::string>::iterator vsit;
    for (vsit = m_messLines.begin(); vsit != m_messLines.end(); vsit++)
    {
        lines.push_back(*vsit + m_messLineSep);
    }
    return lines.size();
}
 
 
 
 
// 就是比较两个字符串头部是否相同
bool MessParser::startsWith(const std::string &src, const std::string &str)
{
    int srcpos = 0;
    int srclen = src.length(); // 74
    int sublen = str.length(); // 6
    if (srclen < sublen)
    {
        return false;
    }
    return (0 == src.compare(srcpos, sublen, str));
}

bool MessParser::endsWith(const std::string &src, const std::string &str)
{
    int srcpos = 0;
    int srclen = src.length();
    int sublen = str.length();
    if (srclen < sublen)
    {
        return false;
    }
    srcpos = srclen - sublen;
    return (0 == src.compare(srcpos, sublen, str));
}

std::string MessParser::replace(const std::string &raw, const std::string &oldstr, const std::string &newstr)
{
    std::string res_string = raw;
    size_t startpos = 0;
    size_t retpos = 0;
    while (std::string::npos != (retpos = res_string.find(oldstr, startpos)))
    {
        if (oldstr.size() == newstr.size())
        {
            (void)res_string.replace(retpos, oldstr.size(), newstr);
        }
        else
        {
            (void)res_string.erase(retpos, oldstr.size());
            (void)res_string.insert(retpos, newstr);
        }
        startpos = retpos + oldstr.size();
    }
    return res_string;
}

// 根据delim拆分line，存入vstr
size_t MessParser::split(const std::string &line, const std::string &delim, std::vector<std::string> &vstr)
{
    size_t pstart = 0;
    size_t phit = 0;
    std::string sstr;
    size_t length = line.length();

    vstr.clear();
    for (; pstart <= length;)
    {
        phit = line.find(delim, pstart); // delim是",",phit是，在原报文中的位置
        if (std::string::npos != phit)
        {
            /* find delim, get substr */
            sstr = line.substr(pstart, phit - pstart); // 切开
            vstr.push_back(sstr);                      // 存入堆栈
            pstart = phit + delim.size();
        }
        else
        {
            /* not find delim, append remaining str and break */
            vstr.push_back(line.substr(pstart));
            break;
        }
    }
    return vstr.size();
}
 
% !Mode:: "TeX:UTF-8"
\NeedsTeXFormat{LaTeX2e}[1999/12/01]
\ProvidesClass{WHUBachelor}[2016/05/31 4.0 Wuhan University Thesis Template for Bachelor Degree]
\LoadClass[zihao=-4,a4paper,oneside]{ctexbook}
\setmainfont{Times New Roman}

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{ctexbook}}
\ProcessOptions
\newif\ifprint\printfalse
\DeclareOption{forprint}{\printtrue}
\ProcessOptions



%------------------------ Page layout ----------------------------------
\RequirePackage{fancyhdr,hyperref}

\hypersetup{pdfencoding=auto}  %%% 邓东升修改
 \ifprint
 \hypersetup{hidelinks}
 \else
\hypersetup{citecolor=magenta,linkcolor=blue}
\fi
\RequirePackage[top=2.5cm,bottom=2.5cm,left=3cm,right=2.5cm,includefoot,xetex]{geometry}    % 页面设置%
\pagestyle{fancyplain}
\fancyhf{}  %清除以前对页眉页脚的设置
\renewcommand{\headrulewidth}{0pt}
\fancyfoot[C]{-\,\thepage\,-}


\fancypagestyle{plain}
{
\fancyhead{}                                    % clear all header fields
\renewcommand{\headrulewidth}{0pt}
\fancyfoot{}                                    % clear all footer fields
\fancyfoot[C]{-\,\thepage\,-}
}
% 行距
\RequirePackage{setspace}
\linespread{1.25}
% 图文间距
\setlength\textfloatsep{10pt}
\setlength\abovecaptionskip{8pt}
\setlength\belowcaptionskip{6pt}
% 设置深度
\setcounter{tocdepth}{4}
\setcounter{secnumdepth}{3}

%value
\newtoks\StudentNumber       %学号
\newtoks\miji            %密级
\newtoks\Cmajor          %学科专业中文名
\newtoks\Emajor          %学科专业英文名
\newtoks\Edate           %英文日期
\newtoks\Etitle          %英文题目
\newtoks\Eauthor         %作者英文名
\newtoks\Esupervisor     %指导教师英文名
\newtoks\Csupervisor     %指导教师中文名
\newtoks\Eschoolname      %学院英文名
\newtoks\Cschoolname      %学院中文名
\newtoks\Grade



%%=====封面===================== 
\def\maketitle{%
  \null
  \thispagestyle{empty}%
  %\vspace*{-2cm}
  \begin{center}\leavevmode
    \includegraphics[width=10cm]{figures/swpu_txt.png}
    \par
    \vspace*{1cm} %插入空白
    \textbf{\songti\fontsize{40}{20}{本科毕业设计（论文）}}\\
    \vspace{1cm}
    \centering
    \includegraphics[width=5cm]{figures/swpu.png}
    \par
    \vspace*{1cm}
    {\heiti\large \zihao{2} \@title \par}%
%    \vspace{4cm}
     \vfill\vfill
    {\songti\zihao{-3}
    \begin{tabular}{cp{5cm}c}
      \raisebox{-3ex}[0pt]{\makebox[3.5cm][s]{院\hfill (系)\hfill 名\hfill 称：}} & {  {}\raisebox{-3ex}[0pt]{\the\Cschoolname}\hfill{}} & \\[3ex]
      \raisebox{-3ex}[0pt]{\makebox[3.5cm][s]{专\hfill 业\hfill 名\hfill 称：}} & { {}\raisebox{-3ex}[0pt]{\the\Cmajor}\hfill{}} & \\[3ex]
      \raisebox{-3ex}[0pt]{\makebox[3.5cm][s]{学\hfill 生\hfill 姓\hfill 名：}} & { {}\raisebox{-3ex}[0pt]{\@author}\hfill{}} & \\[3ex]
      \raisebox{-3ex}[0pt]{\makebox[3.5cm][s]{学\hfill\hfill\hfill 号：}} & { {}\raisebox{-3ex}[0pt]{\the\StudentNumber}\hfill{}} & \\[3ex]
      \raisebox{-3ex}[0pt]{\makebox[3.5cm][s]{指\hfill 导\hfill 教\hfill 师：}} & { {}\raisebox{-3ex}[0pt]{\the\Csupervisor}\hfill{}} & \\[3ex]
     \end{tabular}
    }
    \par
    \vspace{1.5cm}
    {
    {\songti \zihao{3} \@date \par}%
    }
    %\vspace*{-0.7cm}
  \end{center}%
%  \null
%   \cleardoublepage
  }

%------------------------ Abstract & Keywords -------------------------
\newcommand{\abstractofcn}[2]
{
  {\chapter*{\zihao{3}\songti\bf 摘要}}
  \zihao{-4}\songti{#1}\\
  {\zihao{-4}\songti\bf{关键词：}}\zihao{-4}\songti #2
}
\newcommand{\abstractofen}[2]
{
  {\chapter*{\zihao{3}\bf{Abstract}}}
  {\zihao{-4} #1}\\
  {\zihao{-4}\bf{Key words:}} \zihao{-4} #2
}
  

%%------------------------常用宏包-----------------------------------
\RequirePackage{amsmath,amssymb}
\RequirePackage[amsmath,thmmarks]{ntheorem}% AMSLaTeX宏包.
\RequirePackage{graphicx}                 % 图形
\RequirePackage{color,xcolor}             %支持彩色
%\RequirePackage{mathrsfs}   % 不同于\mathcal or \mathfrak 之类的英文花体字体
\RequirePackage{url}
\RequirePackage{enumerate}
\RequirePackage{algorithm}
\RequirePackage{algpseudocode}
\RequirePackage{listings}
\RequirePackage{fontspec}
\RequirePackage{multirow}
\RequirePackage[section]{placeins}
\RequirePackage{longtable}
\RequirePackage{booktabs}
\RequirePackage{titletoc}
\RequirePackage{listings}
%%----------------------- Theorems -----------------------------------
\theoremstyle{plain}
\theoremheaderfont{\heiti}
\theorembodyfont{\songti} \theoremindent0em
\theorempreskip{0pt}
\theorempostskip{0pt}
\theoremnumbering{arabic}
%\theoremsymbol{} %定理结束时自动添加的标志
\newtheorem{theorem}{\hspace{2em}定理}[section]
\newtheorem{definition}{\hspace{2em}定义}[section]
\newtheorem{lemma}{\hspace{2em}引理}[section]
\newtheorem{corollary}{\hspace{2em}推论}[section]
\newtheorem{proposition}{\hspace{2em}性质}[section]
\newtheorem{example}{\hspace{2em}例}[section]
\newtheorem{remark}{\hspace{2em}注}[section]

\theoremstyle{nonumberplain}
\theoremheaderfont{\heiti}
\theorembodyfont{\normalfont \rm \songti}
\theoremindent0em \theoremseparator{\hspace{1em}}
\theoremsymbol{$\square$}
\newtheorem{proof}{\hspace{2em}证明}


\newcommand{\upcite}[1]{\textsuperscript{\cite{#1}}}  %自定义新命令\upcite, 使参考文献引用以上标出现

% 引用编号
\renewcommand{\thetable}{\arabic{section}.\arabic{table}}
\renewcommand{\thefigure}{\arabic{section}.\arabic{figure}}
\renewcommand{\theequation}{\arabic{section}.\arabic{equation}}

%%%%%%%-------------------------------------------------
%%%===  suppress extra inter-line spacing in \list environments
\makeatletter \setlength\partopsep{0pt}
\def\@listI{\leftmargin\leftmargini
            \parsep 0pt
            \topsep \parsep
            \itemsep \parsep}
\@listI
\def\@listii {\leftmargin\leftmarginii
              \labelwidth\leftmarginii
              \advance\labelwidth-\labelsep
              \parsep    \z@ \@plus\z@  \@minus\z@
              \topsep    \parsep
              \itemsep   \parsep}
\def\@listiii{\leftmargin\leftmarginiii
              \labelwidth\leftmarginiii
              \advance\labelwidth-\labelsep
              \parsep    \z@ \@plus\z@  \@minus\z@
              \topsep    \parsep
              \itemsep   \parsep}
\def\@listiv{\leftmargin\leftmarginiv
              \labelwidth\leftmarginiv
              \advance\labelwidth-\labelsep
              \parsep    \z@ \@plus\z@  \@minus\z@
              \topsep    \parsep
              \itemsep   \parsep}
\def\@listv{\leftmargin\leftmarginv
              \labelwidth\leftmarginv
              \advance\labelwidth-\labelsep
              \parsep    \z@ \@plus\z@  \@minus\z@
              \topsep    \parsep
              \itemsep   \parsep}
\def\@listvi{\leftmargin\leftmarginvi
              \labelwidth\leftmarginvi
              \advance\labelwidth-\labelsep
              \parsep    \z@ \@plus\z@  \@minus\z@
              \topsep    \parsep
              \itemsep   \parsep}
%
% Change default margins for \list environments
\setlength\leftmargini   {2\ccwd} \setlength\leftmarginii
{\leftmargini} \setlength\leftmarginiii {\leftmargini}
\setlength\leftmarginiv  {\leftmargini} \setlength\leftmarginv
{\ccwd} \setlength\leftmarginvi  {\ccwd} \setlength\leftmargin
{\leftmargini} \setlength\labelsep      {.5\ccwd}
\setlength\labelwidth    {\leftmargini}
%
\setlength\listparindent{2\ccwd}
% Change \listparindent to 2\ccwd in \list
\def\list#1#2{\ifnum \@listdepth >5\relax \@toodeep
     \else \global\advance\@listdepth\@ne \fi
  \rightmargin \z@ \listparindent2\ccwd \itemindent \z@
  \csname @list\romannumeral\the\@listdepth\endcsname
  \def\@itemlabel{#1}\let\makelabel\@mklab \@nmbrlistfalse #2\relax
  \@trivlist
  \parskip\parsep \parindent\listparindent
  \advance\linewidth -\rightmargin \advance\linewidth -\leftmargin
  \advance\@totalleftmargin \leftmargin
  \parshape \@ne \@totalleftmargin \linewidth
  \ignorespaces}

\makeatother
%---------------------------------------------------
\newcommand\acknowledgement
{\newpage
\addcontentsline{toc}{section}{致谢}
\chapter*{致谢}
}

%%% ---- 目录设置 ----- %%%%
\renewcommand{\contentsname}{\bf \songti \zihao{3} \hspace*{\fill}目\quad 录\hspace*{\fill} \vspace{5pt}}
% 一级标题：顶格、宋体、三号)
\titlecontents{section}[0em]
{\color{black}\zihao{-4}\songti \addvspace{0.5ex}}
{\thecontentslabel\hspace*{1em}} {}
{\titlerule*[0.4pc]{.}\contentspage}
% 二级标题：右缩两个汉字空格、宋体、小三号
\titlecontents{subsection}[2em]
{\color{black}\zihao{-4}\songti \addvspace{0.5ex}}
{\thecontentslabel\hspace*{1em}} {\hspace*{-2.3em}}
{\titlerule*[0.4pc]{.}\contentspage}
% 三级标题：右缩四个汉字空格、宋体、四号
\titlecontents{subsubsection}[4em] 
{\color{black}\zihao{-4}\songti \addvspace{0.5ex}}
{\thecontentslabel\hspace*{1em}} {\hspace*{-2.3em}}
{\titlerule*[0.4pc]{.}\contentspage}

%%% ---- 章节设置 ----- %%%%
\ctexset{chapter={format+={\bf\zihao{3}\songti},number={},name={,},afterskip={17pt},beforeskip={16.5pt}}}
\ctexset{section={format+={\bf\zihao{3}\songti\raggedright},number={\arabic{section}},afterskip={17pt},beforeskip={16.5pt}}}
\ctexset{subsection={format+={\bf\zihao{-3}\songti\raggedright},number={\arabic{section}.\arabic{subsection}},afterskip={13pt},beforeskip={13pt}}}
\ctexset{subsubsection={format+={\bf\zihao{4}\songti\raggedright},number={\arabic{section}.\arabic{subsection}.\arabic{subsubsection}},afterskip={6pt},beforeskip={6pt}}}

%%% ---- 代码设置 ---- %%%%

\renewcommand{\algorithmicrequire}{\textbf{输入:}}
\renewcommand{\algorithmicensure}{\textbf{输出:}}
\floatname{algorithm}{算法}
\newfontfamily\menlo{Menlo}
\definecolor{mygreen}{rgb}{0,0.6,0}
\definecolor{mygray}{rgb}{0.5,0.5,0.5}
\definecolor{mymauve}{rgb}{0.58,0,0.82}
\lstset{
    backgroundcolor     =   \color{white},
    basicstyle          =   \sffamily,          % 基本代码风格
    keywordstyle        =   \color{blue},          % 关键字风格
    commentstyle        =   \color{mygreen},  % 注释的风格，斜体
    stringstyle         =   \color{mymauve}\ttfamily,  % 字符串风格
    flexiblecolumns,                % 别问为什么，加上这个
    numbers             =   left,   % 行号的位置在左边
    showspaces          =   false,  % 是否显示空格，显示了有点乱，所以不现实了
    numberstyle         =   \zihao{-5}\ttfamily,    % 行号的样式，小五号，tt等宽字体
    showstringspaces    =   false,
    captionpos          =   b,      % 这段代码的名字所呈现的位置，t指的是top上面
    frame               =   lrtb,   % 显示边框
    columns             =   fullflexible,
    tabsize             =   4,
    breaklines          =   true, 
    rulesepcolor        =   \color{red!20!green!20!blue!20}
}

\lstdefinestyle{C++}{
    language        =   C++, % 语言
    basicstyle      =   \zihao{-5}\ttfamily,
    numberstyle     =   \zihao{-5}\ttfamily,
    breaklines      =   true,   % 自动换行，建议不要写太长的行
    columns         =   fixed,  % 如果不加这一句，字间距就不固定，很丑，必须加
    basewidth       =   0.5em,
}


%%% ---- 图表标题设置 ----- %%%%
\RequirePackage[labelsep=quad]{caption} %% 序号之后空一格写标题
\captionsetup[table]{font={small,bf}}  %%设置表格标题字体为黑体  。2016.06.07
\captionsetup[figure]{font={small,bf}}
\captionsetup[lstlisting]{font={small,bf}}
\renewcommand\figurename{\bf\zihao{5} 图}
\renewcommand\tablename{\bf\zihao{5} 表} 
\renewcommand\lstlistingname{\bf\zihao{5} 代码}

%%% ------ 目录设置 ------- %%%%
%\ctexset{contentsname={目\qquad 录}}
 
%\RequirePackage{tocloft} 
%\renewcommand\cftchapfont{\zihao{4}\bf}
%\renewcommand\cfttoctitlefont{\hfill\zihao{-2}\bf}
%\renewcommand\cftaftertoctitle{\hfill}
 
%%===参考文献===%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\bibliographystyle{abbrv}        % 参考文献样式,  plain,unsrt,alpha,abbrv,chinesebst 等等
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\graphicspath{{figures/}}        % 图片文件路径
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\allowdisplaybreaks

\endlinechar `\^^M
\endinput

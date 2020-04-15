#include <cgic.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void ShowForm();
void File();
void HandleSubmit();

int cgiMain() {
	/* Send the content type, letting the browser know this is HTML */
	cgiHeaderContentType("text/html");
	/* Top of the page */
	fprintf(cgiOut, "<HTML><HEAD>\n");
	fprintf(cgiOut, "<TITLE>Submit Your Gcode</TITLE></HEAD>\n");
	fprintf(cgiOut, "<BODY><H1>Submit Your Gcode</H1>\n");
	/* If a submit button has already been clicked, act on the
		submission of the form. */
	if ((cgiFormSubmitClicked("testcgic") == cgiFormSuccess))
	{
		HandleSubmit();
		fprintf(cgiOut, "<hr>\n");
	}
	/* Now show the form */
	ShowForm();
	/* Finish up the page */
	fprintf(cgiOut, "</BODY></HTML>\n");
	return 0;
}

void HandleSubmit()
{
	File();
}

void File()
{
	cgiFilePtr file;
	char name[40];
	char buffer[1024];
	int size;
	int got;
	FILE *fp;
	char * dir = "/home/root/";
	if (cgiFormFileName("file", name, sizeof(name)) != cgiFormSuccess) {
		printf("<p>No file was uploaded.<p>\n");
		return;
	}

	char fileSave[80];
	strcpy(fileSave, dir);
	strcat(fileSave, name);
	fp = fopen(fileSave, "wb");

	fprintf(cgiOut, "The gcode file was: ");
	cgiHtmlEscape(name);
	fprintf(cgiOut, "<p>\n");
	cgiFormFileSize("file", &size);
	fprintf(cgiOut, "The gcode size was: %d bytes<p>\n", size);
	fprintf(cgiOut, "<p>\n");
	fprintf(cgiOut, "The gcode was saved to %s<p>\n", fileSave);
	fprintf(cgiOut, "<p>\n");
	fprintf(cgiOut, "The gcode was: ");
	if (cgiFormFileOpen("file", &file) != cgiFormSuccess) {
		fprintf(cgiOut, "Could not open the file.<p>\n");
		return;
	}
	fprintf(cgiOut, "<pre>\n");
	while (cgiFormFileRead(file, buffer, sizeof(buffer), &got) ==
		cgiFormSuccess)
	{
		cgiHtmlEscapeData(buffer, got);
		fwrite(buffer, 1, got, fp);
	}
	fprintf(cgiOut, "</pre>\n");
	cgiFormFileClose(file);
	fclose(fp);
}

void ShowForm()
{
	fprintf(cgiOut, "<!-- 2.0: multipart/form-data is required for file uploads. -->");
	fprintf(cgiOut, "<form method=\"POST\" enctype=\"multipart/form-data\" ");
	fprintf(cgiOut, "	action=\"");
	cgiValueEscape(cgiScriptName);
	fprintf(cgiOut, "\">\n");
	fprintf(cgiOut, "<p>File Upload:\n");
	fprintf(cgiOut, "<input type=\"file\" name=\"file\" value=\"\"> (Select A Local File)\n");
	fprintf(cgiOut, "<p>\n");
	fprintf(cgiOut, "<input type=\"submit\" name=\"testcgic\" value=\"Submit Request\">\n");
	fprintf(cgiOut, "</form>\n");
}

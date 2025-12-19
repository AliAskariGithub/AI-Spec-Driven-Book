import React from 'react';
import OriginalDocPage from '@theme-original/DocPage';
import TypographyWrapper from '../components/TypographySystem';

// This wrapper applies the typography system to all doc pages (textbook content)
export default function DocPage(props) {
  return (
    <TypographyWrapper variant="body" pageType="textbook">
      <OriginalDocPage {...props} />
    </TypographyWrapper>
  );
}
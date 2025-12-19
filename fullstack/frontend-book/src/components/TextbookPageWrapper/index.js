import React from 'react';
import Layout from '@theme/Layout';
import TypographyWrapper from '../TypographySystem';

const TextbookPageWrapper = ({ children, ...props }) => {
  return (
    <Layout {...props} className="textbook-page">
      <TypographyWrapper variant="body" pageType="textbook">
        {children}
      </TypographyWrapper>
    </Layout>
  );
};

export default TextbookPageWrapper;